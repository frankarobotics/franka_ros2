#!groovy
pipeline {

  libraries {
    lib('fe-pipeline-steps')
  }

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
           defaultValue: 'main',
           description: 'Branch or tag to checkout for franka_description. Defaults to the same ROS_DISTRO branch of franka_ros2.')

    booleanParam(name: 'executeHardwareTestsOnRobot',
           defaultValue: true,
           description: "Run the franka_ros2 tests on the real hardware")

    string(name: "robotIp",
            defaultValue: "172.16.0.1",
            description: "The static IP of the robot to run tests onto")
  }

  environment {
    HOME="${WORKSPACE}" // for cleaning up ros logs
  }


  stages {
    stage('Get Ready') {
      options { skipDefaultCheckout true }
      steps {
        script{
          cleanWs()
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
      // re-enable this when https://franka.atlassian.net/browse/BFFTRAC-2632 is fixed
      // when { expression { params.executeHardwareTestsOnRobot } }
      when { expression { false } }
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
          sh """
            . install/setup.sh
            chmod +x ./src/franka_ros2/scripts/*.sh
            ./src/franka_ros2/scripts/run_hardware_tests.sh ${params.robotIp}
          """
        }
      }
    }
  }
  post {
      always {
          script {
              cleanWs()
              notifyBitbucket()
          }
      }
  }
}
