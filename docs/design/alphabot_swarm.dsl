workspace {
    // Allow reference to identified using . notation
    !identifiers hierarchical

    model {
        // Entities
        operator = person "Swarm Operator"

        mother_computer = softwareSystem "Mother Computer" "A powerful computer that issues formation commanding" {
            cosmos_agent = container "COSMOS Agent" "Service that acts as the swarm-awareness manager. Written in C++."
        }

        group "Alphabot Swarm" {
            alphabot = softwareSystem "Alphabot" "A mini-driving robot acting as a single node in the swarm" {
                cosmos_agent = container "COSMOS Agent" "Service that acts as the swarm-awareness manager. Written in C++." {
                    mac = component "Multi-Agent Coordinator (MAC)" "Outputs position, velocity, acceleration needed to achieve the desired formation"
                    robot_manager = component "Robot Manager" "Manages communications with the Python service that controls the robot."
                    
                    // Relations
                    mac -> robot_manager "Sends command to move to some specified position"
                    robot_manager -> mac "Gives robot's reported position"
                }
                python_service = container "Robot Service" "Service that provides an interface for driving the robot. Written in Python." {
                    robot_manager = component "Robot Manager" "Manages the state and main functionalities required by the robot"
                    api_manager = component "API Manager" "Manages the interfaces provided by the service for this robot"
                    position_estimator = component "Position Estimator" "Provides estimates of the robot's position based on dead reckoning"
                    driver_manager = component "Driver Manager" "Uses the robot's driver libraries to physically control the robot"
                    
                    // Relations
                    api_manager -> robot_manager "Relays incoming command"
                    robot_manager -> driver_manager "Commands to move to a specified location"
                    robot_manager -> position_estimator "Informs about most recent positioning command"
                    position_estimator -> robot_manager "Reports estimated position"
                    robot_manager -> api_manager "Sends telemetry"
                }
                // Relations
                cosmos_agent.robot_manager -> python_service.api_manager "Sends command to move to some specified position"
                python_service.api_manager -> cosmos_agent.robot_manager "Reports estimated position"
            }
            alphabot_n = softwareSystem "Alphabot n" "Nth node in the swarm"
        }
  
        // Relations
        operator -> mother_computer "Uses"
        mother_computer -> alphabot "Commands to move to desired position"
        alphabot -> mother_computer "Reports current estimated position"
        mother_computer.cosmos_agent -> alphabot.cosmos_agent "Sends command to move to some specified position via PacketComm"
        alphabot.cosmos_agent -> mother_computer.cosmos_agent "Sends robot telemetry"

        
    }

    views {
        systemLandscape "OverallSystem" {
            include *
        }
        container alphabot "Alphabot" {
            include *
        }
        component alphabot.cosmos_agent "COSMOS_Agent" {
            include *
        }
        component alphabot.python_service "PythonService" {
            include *
        }
        container mother_computer "MotherComputer" {
            include *
        }
        
        styles {
            element "Software System" {
                background SteelBlue
            }
            element "Container" {
                background LightSteelBlue
            }
            element "Person" {
                shape person
            }
        }
    }
    
}
