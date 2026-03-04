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
# Run Hardware Tests
# ---------------------------------------------------------------------------
echo "Running franka_ros2 hardware tests..."
rm -f reports/*.xml
colcon test \
    --base-paths src \
    --packages-select franka_bringup \
    --event-handlers console_direct+ \
    --ctest-args --tests-regex test_hardware
colcon test-result --verbose
TEST_EXIT_CODE=$?

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

if [ $TEST_EXIT_CODE -ne 0 ]; then
    exit 1
fi