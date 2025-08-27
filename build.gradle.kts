plugins {
   id("us.ihmc.ihmc-build")
}

ihmc {
   group = "us.ihmc"
   version = "0.2"
   vcsUrl = "https://github.com/stephenmcc/mcts-lipm-walker"
   openSource = true

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:euclid:0.22.3")
   api("us.ihmc:euclid-geometry:0.22.3")
   api("us.ihmc:euclid-frame:0.22.3")
   api("us.ihmc:euclid-shape:0.22.3")
   api("us.ihmc:euclid-frame-shape:0.22.3")
   api("us.ihmc:ihmc-realtime:1.7.0")
   api("us.ihmc:ihmc-ros-control:0.7.1")
   api("us.ihmc:ihmc-footstep-planning:source")
   api("us.ihmc:ihmc-avatar-interfaces:source")
}