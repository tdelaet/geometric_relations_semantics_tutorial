-- deploy_app.lua
tc = rtt.getTC()
depl = tc:getPeer("Deployer")
depl:loadService("lua", "marshalling")
 
-- import components, requires correctly setup RTT_COMPONENT_PATH or
-- ROS_PACKAGE_PATH
depl:import("ocl")
depl:import("geometric_relations_semantics_tutorial_orocos_components")
depl:import("rtt_geometry_msgs")

-- write properties
writeProps=tc:provides("marshalling"):getOperation("writeProperties")

-- create components
depl:loadComponent("subscriber", "Geometric_semantics_component_tutorial_subscriber")
depl:loadComponent("publisher", "Geometric_semantics_component_tutorial_publisher")

-- loading service marshalling
depl:loadService("subscriber", "marshalling")
depl:loadService("publisher", "marshalling")

-- get reference to new peer
subscriberComponent= depl:getPeer("subscriber")
publisherComponent= depl:getPeer("publisher")

-- create ros topics
depl:stream("publisher.outPortPose", rtt.provides("ros"):topic("/geometric_semantics/pose"))

-- read properties
publisherComponent:provides("marshalling"):readProperties("props_publisher.cpf")
subscriberComponent:provides("marshalling"):readProperties("props_subscriber.cpf")

-- configure
publisherComponent:configure()
subscriberComponent:configure()

-- create activity for producer: period=1, priority=0,
-- schedtype=ORO_SCHED_OTHER (1).
depl:setActivity("publisher", 1, 0, rtt.globals.ORO_SCHED_RT)
 
-- raise loglevel
rtt.setLogLevel("Error")

-- start
publisherComponent:start()
