#!/bin/bash

# Configuration
if [ $# -eq 0 ]; then
    echo "Usage: $0 <robot_ip>"
    exit 1
fi

ROBOT_IP=$1
BASE_URL="https://$ROBOT_IP"
CREDENTIALS="franka:franka123"
TOKEN=""

# ---------------------------------------------------------------------------
# Helper Function
# ---------------------------------------------------------------------------
call_api() {
    local endpoint=$1
    local payload=$2
    local extra_headers=()

    # If we have a token, add the header
    if [ ! -z "$TOKEN" ]; then
        extra_headers+=(-H "X-Control-Token: $TOKEN")
    fi

    # Perform the request
    curl -s -k -X 'POST' \
        "$BASE_URL$endpoint" \
        -u "$CREDENTIALS" \
        -H 'accept: application/json;charset=utf-8' \
        -H 'Content-Type: application/json;charset=utf-8' \
        "${extra_headers[@]}" \
        -d "$payload"
}

# ---------------------------------------------------------------------------
# Main Execution Steps
# ---------------------------------------------------------------------------

echo "[1/6] Requesting Control Token..."
RESPONSE=$(call_api "/api/system/control-token:take" '{ "owner": "franka" }')

# Extract token
TOKEN=$(echo "$RESPONSE" | python3 -c "import sys, json; print(json.load(sys.stdin)['token'])")

if [ -z "$TOKEN" ]; then
    echo "Error: Failed to acquire token. Response: $RESPONSE"
    exit 1
fi
echo "      -> Token acquired: $TOKEN"

echo "[2/6] Unlocking Joints..."
call_api "/api/arm/joints:unlock" '' 
echo "      -> Joints unlocked."

echo "[3/6] Activating FCI..."
call_api "/api/fci:activate" ''
echo "      -> FCI Active."
sleep 5

# ---------------------------------------------------------------------------
# Run Hardware Tests with Retry Logic
# ---------------------------------------------------------------------------
echo "Running franka_ros2 hardware tests..."

MAX_RETRIES=2  # Only retry once if constraint violations detected
RETRY_DELAY=5  # seconds between retries
TEST_SUCCESS=false

for attempt in $(seq 1 $MAX_RETRIES); do
    echo "Test attempt $attempt/$MAX_RETRIES..."
    
    rm -f reports/*.xml
    colcon test \
        --base-paths src \
        --packages-select franka_bringup \
        --event-handlers console_direct+ \
        --ctest-args --tests-regex test_hardware
    colcon test-result --verbose
    TEST_EXIT_CODE=$?
    
    if [ $TEST_EXIT_CODE -eq 0 ]; then
        echo "Tests passed on attempt $attempt"
        TEST_SUCCESS=true
        break
    else
        echo "Tests failed on attempt $attempt (exit code: $TEST_EXIT_CODE)"
        
        # Check if this is a retriable failure (communication constraints violation)
        if colcon test-result --verbose | grep -q "communication.*constraint"; then
            if [ $attempt -lt $MAX_RETRIES ]; then
                echo "Detected communication constraint violation. Retrying in $RETRY_DELAY seconds..."
                sleep $RETRY_DELAY
                continue
            else
                echo "Max retries reached. Giving up."
            fi
        else
            echo "Non-retriable failure detected. Not retrying."
            break
        fi
    fi
done

# ---------------------------------------------------------------------------
# Teardown
# ---------------------------------------------------------------------------
echo "[4/6] Deactivating FCI..."
call_api "/api/fci:deactivate" ''
sleep 1

echo "[5/6] Locking Joints..."
call_api "/api/arm/joints:lock" ''

echo "[6/6] Releasing Control Token..."
call_api "/api/system/control-token:release" ''

if [ "$TEST_SUCCESS" = false ]; then
    exit 1
fi