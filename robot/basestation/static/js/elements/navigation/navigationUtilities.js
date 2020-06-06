/*
This is a piece of code created by @Waleed-HM which I converted to JS
*/


function directionToRover(p1_lat, p1_lon, p2_lat, p2_lon) {
    /*
    This is a function to calculate the direction from p1 (point1) to p2 (point2)
    It uses the Haverside formulas
    Inputs are p1 and p2 latitudes and longitudes in degrees
    Output is the direction from p1 to p2 in degrees ranging from 0 to 360
    Measured from the N-S line, left-handed rule is used (E=90, W=270)
    */

    p1_lat = p1_lat * Math.PI / 180
    p1_lon = p1_lon * Math.PI / 180
    p2_lat = p2_lat * Math.PI / 180
    p2_lon = p2_lon * Math.PI / 180

    let delta_lat = p2_lat - p1_lat
    let delta_lon = p2_lon - p1_lon

    let X = Math.cos(p2_lat) * Math.sin(delta_lon)
    let Y = Math.cos(p1_lat) * Math.sin(p2_lat) - Math.sin(p1_lat) * Math.cos(p2_lat) * Math.cos(delta_lon)

    let realDir = Math.atan2(X, Y)
    realDir = realDir * 180 / Math.PI

    if (realDir < 0) {
        realDir += 360
    }
    return realDir
}


function distanceToRover(p1_lat, p1_lon, p2_lat, p2_lon) {
    /*
    This is a function to calculate the distance between p1 (point1) to p2 (point2)
    It uses the Haverside formulas
    Inputs are p1 and p2 latitudes and longitudes in degrees
    Output is the distance between p1 and p2 in meters
    */

    let R = 6371000; //radius of Earth

    p1_lat *= Math.PI / 180
    p1_lon *= Math.PI / 180
    p2_lat *= Math.PI / 180
    p2_lon *= Math.PI / 180

    let delta_lat = p2_lat - p1_lat
    let delta_lon = p2_lon - p1_lon

    let A = Math.pow(Math.sin(delta_lat / 2), 2) + Math.cos(p2_lat) * Math.cos(p1_lat) * Math.pow(Math.sin(delta_lon / 2), 2)
    let B = 2 * Math.atan2(Math.sqrt(A), Math.sqrt(1 - A))
    return (R * B)
}
