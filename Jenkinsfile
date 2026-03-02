#!groovy
@Library('fe-pipeline-steps@feat/robot_api')
import de.franka.jenkins.FeRobotApi

pipeline {

  agent {
    node {
      label 'fepc-lx010'
    }
  }

  triggers {
    pollSCM('H/5 * * * *')
  }

  parameters {
    string(name: 'libfrankaRepoUrl',
           defaultValue: 'ssh://git@bitbucket.fe.lan:7999/moctrl/libfranka.git',
           description: 'SSH URL to clone libfranka')

    string(name: 'libfrankaBranch',
           defaultValue: 'main',
           description: 'Branch or tag to checkout for libfranka.')

    string(name: 'frankaDescriptionRepoUrl',
           defaultValue: 'ssh://git@bitbucket.fe.lan:7999/moctrl/franka_description.git',
           description: 'SSH URL to clone franka_description.')

    string(name: 'frankaDescriptionBranch',
           defaultValue: 'jazzy',
           description: 'Branch or tag to checkout for franka_description.')

    booleanParam(name: 'executeHardwareTestsOnRobot',
           defaultValue: true,
           description: "Run the franka_ros2 tests on the real hardware")

    string(name: "robotIp",
            defaultValue: "172.16.0.1",
            description: "The static IP of the robot to run tests onto")

    booleanParam(name: 'cleanWorkspace', 
            defaultValue: false, 
            description: 'Wipe entire workspace before build')
  }


  stages {
    stage('Get Ready') {
      options { skipDefaultCheckout true }
      steps {
        script{
          if (params.cleanWorkspace) {
            cleanWs()
          }
        }
        checkout([
            $class: 'GitSCM',
            branches: scm.branches,
            userRemoteConfigs: scm.userRemoteConfigs,
            extensions: [
                [$class: 'RelativeTargetDirectory', relativeTargetDir: 'src'] // needed to put everything under src as expected by colcon/rosdep
            ]
        ])
        script {
          notifyBitbucket()
          currentBuild.displayName = "[libfranka: ${params.libfrankaBranch}, franka_description: ${params.frankaDescriptionBranch}]"
        }
      }
    }

    stage('Fetch Dependencies') {
      options { skipDefaultCheckout true }
      steps {
        sh 'echo "=== Workspace structure ===" && ls -la'
        dir('src') {
          script {

            def repos = readYaml file: 'dependency.repos'
            def repoMap = repos?.repositories ?: [:]

            // Clone/update all 3rd party dependencies 
            repoMap.each {repoName, cfg ->
              def url = cfg.url 
              def version = cfg.version ?: 'main'
              if (repoName != 'libfranka' && repoName != 'franka_description'){
                sh """
                  if [ -d ${repoName} ]; then
                    cd ${repoName}
                    git checkout ${version}
                    git pull
                    cd ..
                  else
                    git clone --depth 1 --branch ${version} ${url} ${repoName}
                  fi
                """
              }
            }

            sshagent(['git_ssh']){
            sh """

              if [ -d "libfranka" ]; then
                cd libfranka 
                git fetch --all --tags
                git checkout ${params.libfrankaBranch}
                git pull
                cd ..
              else
                git clone --branch ${params.libfrankaBranch} ${params.libfrankaRepoUrl}
                cd libfranka 
                git config submodule.common.url ssh://git@bitbucket.fe.lan:7999/moctrl/libfranka-common.git
                git submodule update --init --recursive --depth 1 
                cd ..
              fi

              if [ -d "franka_description" ]; then
                cd franka_description
                git fetch --all --tags
                git checkout ${params.frankaDescriptionBranch}
                git pull
                cd ..
              else
                git clone --depth 1 --branch ${params.frankaDescriptionBranch} ${params.frankaDescriptionRepoUrl}
              fi
              """
            }
          }
          sh 'echo "=== src structure ===" && ls -la'
        }
      }
    }

    stage('Build') {
      options { skipDefaultCheckout true }
      agent {
        dockerfile {
          dir 'src'
          reuseNode true
        }
      }
      environment {
        ROBOT_IP = "${params.robotIp}"
      }
      steps {
        sh '''
          . /opt/ros/$ROS_DISTRO/setup.sh
          echo "=== Workspace structure ===" && ls -la
          colcon build \
            --base-paths src \
            --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON  \
                         -DCHECK_TIDY=ON \
                         -DBUILD_TESTS=OFF \
                         -DBUILD_TESTING=ON  \
                         -DROBOT_IP=$ROBOT_IP
        '''
      }
    }

    // Run the unit tests (no hardware tests)
    stage('Test') {
      options { skipDefaultCheckout true }
      agent {
        dockerfile {
          dir 'src'
          reuseNode true
        }
      }
      steps {
        sh '''
          . install/setup.sh
          colcon test \
            --base-paths src \
            --packages-select-regex '^franka_(?!bringup$|gripper$)' \
            --event-handlers console_direct+ \
            --ctest-args --exclude-regex test_hardware
          colcon test-result --verbose
        '''
      }
      post {
        always {
          junit 'build/**/test_results/**/*.xml'
        }
      }
    }
    stage("Hardware Test"){
      when { expression { params.executeHardwareTestsOnRobot } }
      options { skipDefaultCheckout true }
      agent { 
        dockerfile {
          dir 'src'
          reuseNode true
          args '--network host ' +
               '--cap-add=sys_nice ' +
               '--ulimit rtprio=99 ' +
               '--ulimit rttime=-1 ' +
               '--ulimit memlock=8428281856 ' +
               '--security-opt=seccomp=unconfined'
        }
      }

      steps {
        script {
          echo "Checking connectivity to Master Controller..."
          sh "ping -c 5 ${params.robotIp}"

          def robot = new FeRobotApi(this, params.robotIp)

          try {
            // take control token
            robot.controlTokenStatus()
            robot.controlTokenTake()
            robot.controlTokenStatus()

            sh 'sleep 1'

            // unlock brakes (worth adding some retries)
            robot.jointsStatus()
            robot.jointsUnlock()

            if (!robot.areAllJointsUnlocked()) {
              robot.jointsStatus()
              throw new RuntimeException("Failed to unlock joints")
            }

            sh 'sleep 5'

            // Activate FCI
            robot.fciStatus()
            robot.fciActivate()

            if(!robot.isFciActive()) {
              robot.fciStatus()
              throw new RuntimeException("Failed to activate FCI")
            }

            sh 'sleep 1'

            echo 'Running hardware tests...'
            sh '''
                . install/setup.sh
                colcon test \
                  --base-paths src \
                  --packages-select franka_bringup \
                  --event-handlers console_direct+ \
                  --ctest-args --tests-regex test_hardware

                colcon test-result --verbose
            '''
          }
          finally {
            try {
                robot.fciDeactivate()
                sh 'sleep 1'
                robot.jointsLock()
                sh 'sleep 5'
                robot.controlTokenRelease()

            }
            catch(Exception cleanupErr){
                println "Cleanup warning: ${cleanupErr.message}"
            }
          }
        }
      }
    }
  }
  post {
      always {
          script {
              notifyBitbucket()
          }
      }
  }
}
