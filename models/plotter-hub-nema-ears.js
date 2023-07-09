const jscad = require('@jscad/modeling')
const { cylinder, polygon, cuboid, roundedCuboid } = jscad.primitives
const { translate, rotateZ, translateZ, translateY, rotateX,rotateY, translateX } = jscad.transforms
const { extrudeLinear, extrudeRotate } = jscad.extrusions
const { union, subtract } = jscad.booleans
const { vec2 } = jscad.maths
const { degToRad } = jscad.utils
const { cube, ellipse, star, triangle, rectangle } = require('@jscad/modeling').primitives
const { hull, hullChain } = require('@jscad/modeling').hulls

// Main entry point; here we construct our solid:
const main = () => {
  let tilt = translate([-8, 8, 0], triangle({type: "SSS", values: [16, 50, 50]}));
  tilt = union(rectangle({size: [16, 16]}), tilt);
  tilt = extrudeLinear({height: 1.0}, tilt); 
  tilt = hull(tilt, cuboid({size: [16, 2, 8], center: [0, 5, 4]}));
  let notch = cuboid({size:[16, 3, 10]});
  notch = translate([0, 3, 5], notch);
  tilt = union(tilt, notch);
  let motor = translate([0, -10, 0], cylinder({radius: 13/2, height: 20}));
  return subtract(tilt, motor);
}

module.exports = { main}
  
