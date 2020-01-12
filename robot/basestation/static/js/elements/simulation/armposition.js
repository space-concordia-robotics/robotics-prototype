$(document).ready(() => {
  /* URDF visualization */
  // Create the main viewer.
  let viewer = new ROS3D.Viewer({
    divID : 'urdf',
    width : 500,
    height : 300,
    antialias : true
  })

  // Add a grid.
  viewer.addObject(new ROS3D.Grid());

  // Setup a client to listen to TFs.
  let tfClient = new ROSLIB.TFClient({
    ros : ros,
    angularThres : 0.01,
    // the default example uses 0.01 which is not small enough to see the fingers move smoothly
    transThres : 0.001,
    rate : 10.0
  })

  // Setup the URDF client.
  let urdfClient = new ROS3D.UrdfClient({
    ros : ros,
    tfClient : tfClient,
    path : 'http://localhost:5000/static/model',
    rootObject : viewer.scene,
    loader : ROS3D.COLLADA_LOADER_2
  })
})
