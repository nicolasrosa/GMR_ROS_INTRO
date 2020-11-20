function subscriber_callback(msg)
    -- This is the subscriber callback function
    sim.addStatusbarMessage('subscriber receiver following Float32: '..msg.data)
end

function getTransformStamped(objHandle,name,relTo,relToName)
    -- This function retrieves the stamped transform for a specific object
    t=sim.getSystemTime()
    p=sim.getObjectPosition(objHandle,relTo)
    o=sim.getObjectQuaternion(objHandle,relTo)
    return {
        header={
            stamp=t,
            frame_id=relToName
        },
        child_frame_id=name,
        transform={
            translation={x=p[1],y=p[2],z=p[3]},
            rotation={x=o[1],y=o[2],z=o[3],w=o[4]}
        }
    }
end

function sysCall_init()
    -- The child script initialization
    objectHandle=sim.getObjectAssociatedWithScript(sim.handle_self)
    objectName=sim.getObjectName(objectHandle)
    rosInterfacePresent=simROS

    -- Retrieve handles
    leftMotorHandle  = sim.getObjectHandle('left_motor') 
    rightMotorHandle = sim.getObjectHandle('right_motor')
    vehicleHandle    = sim.getObjectHandle('AMR_Full')
    frontSonarHandle = sim.getObjectHandle('front_sonar')
    rightSonarHandle = sim.getObjectHandle('right_sonar')
    leftSonarHandle = sim.getObjectHandle('left_sonar')

    -- Declaring the signals
    -- sim.setFloatSignal('frontSonarSignal',0.0)
    -- sim.setFloatSignal('rightSonarSignal',0.0)
    -- sim.setFloatSignal('leftSonarSignal',0.0)
    -- sim.setFloatSignal('rightMotorSignal',0.0)
    -- sim.setFloatSignal('leftMotorSignal',0.0)

    -- Prepare the float32 publisher and subscriber (we subscribe to the topic we advertise):
    if rosInterfacePresent then
        publisher=simROS.advertise('/simulationTime','std_msgs/Float32')
        subscriber=simROS.subscribe('/simulationTime','std_msgs/Float32','subscriber_callback')
    
        -- Publisher
        frontSonar_pub = simROS.advertise('vehicle/frontSonar','std_msgs/Bool')
        leftSonar_pub = simROS.advertise('vehicle/leftSonar','std_msgs/Bool')
        rightSonar_pub = simROS.advertise('vehicle/rightSonar','std_msgs/Bool')
        -- odometry = simROS.advertise('vehicle/odometry',1,simros_strmcmd_get_odom_data,vehicleHandle,-1,'')
    
        -- Subscriber
        -- motorLeftSpeed_sub = simROS.subscribe('vehicle/motorLeftSpeed','std_msgs/Float32','setLeftMotorVelocity_cb')
        -- motorRightSpeed_sub = simROS.subscribe('vehicle/motorRightSpeed','std_msgs/Float32','setRightMotorVelocity_cb')
motorLeftSpeed_sub = simROS.subscribe('/left_rpm','std_msgs/Float32','setLeftMotorVelocity_cb')
        motorRightSpeed_sub = simROS.subscribe('/right_rpm','std_msgs/Float32','setRightMotorVelocity_cb')
    end
end

function setLeftMotorVelocity_cb(msg)
    -- Left motor speed subscriber callback
    sim.setJointTargetVelocity(leftMotorHandle,msg.data/10)
end

function setRightMotorVelocity_cb(msg)
    -- Right motor speed subscriber callback
    sim.setJointTargetVelocity(rightMotorHandle,msg.data/10)
end

function sysCall_actuation()
    -- Send an updated simulation time message, and send the transform of the object attached to this script:
    if rosInterfacePresent then
        simROS.publish(publisher,{data=sim.getSimulationTime()})

        local  leftSonar_res=sim.readProximitySensor( leftSonarHandle)
        local frontSonar_res=sim.readProximitySensor(frontSonarHandle)
        local rightSonar_res=sim.readProximitySensor(rightSonarHandle)
    
        local  leftSonar_detectionTrigger = {}
        local frontSonar_detectionTrigger = {}
        local rightSonar_detectionTrigger = {}
        
        leftSonar_detectionTrigger['data']  =  leftSonar_res > 0
        frontSonar_detectionTrigger['data'] = frontSonar_res > 0
        rightSonar_detectionTrigger['data'] = rightSonar_res > 0
        
        simROS.publish( leftSonar_pub,  leftSonar_detectionTrigger)
        simROS.publish(frontSonar_pub, frontSonar_detectionTrigger)
        simROS.publish(rightSonar_pub, rightSonar_detectionTrigger)
        
        -- Send the robot's transform:
        simROS.sendTransform(getTransformStamped(objectHandle,objectName,-1,'world'))
        -- To send several transforms at once, use simROS.sendTransforms instead
    end
end

function sysCall_cleanup()
    -- Following not really needed in a simulation script (i.e. automatically shut down at simulation end):
    if rosInterfacePresent then
        simROS.shutdownPublisher(publisher)
        simROS.shutdownSubscriber(subscriber)
        
        simROS.shutdownPublisher(leftSonar_pub)
        simROS.shutdownPublisher(frontSonar_pub)
        simROS.shutdownPublisher(rightSonar_pub)
        simROS.shutdownSubscriber(motorLeftSpeed_sub)
        simROS.shutdownSubscriber(motorRightSpeed_sub)
    end
end