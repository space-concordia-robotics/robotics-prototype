/**
   * Setup all visualization elements when the page is loaded.
   */
  function initR() {
    // Connect to ROS.
    var ros = new ROSLIB.Ros({
      url : 'ws://localhost:9090'
    });

    // Create the main viewer.
    var viewer = new ROS3D.Viewer({
      divID : 'urdf',
      width : 610,
      height : 345,
      antialias : true
    });

    // Add a grid.
    viewer.addObject(new ROS3D.Grid());

    // Setup a client to listen to TFs.
    var tfClient = new ROSLIB.TFClient({
      ros : ros,
      angularThres : 0.01,
      transThres : 0.01,
      rate : 10.0
    });

    // Setup the URDF client.
    var urdfClient = new ROS3D.UrdfClient({
      ros : ros,
      tfClient : tfClient,
      //path : 'http://static.robotwebtools.org/',
      path : 'http://localhost:5000/static/model',
      rootObject : viewer.scene,
      loader : ROS3D.COLLADA_LOADER_2
    });
  }
