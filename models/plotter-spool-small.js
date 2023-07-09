const jscad = require('@jscad/modeling')
const { cylinder, polygon, cuboid } = jscad.primitives
const { rotateZ, translateZ, translateY, rotateX,rotateY, translateX, translate } = jscad.transforms
const { extrudeLinear, extrudeRotate } = jscad.extrusions
const { union, subtract } = jscad.booleans
const { vec2 } = jscad.maths
const { degToRad } = jscad.utils
const { cube, ellipse, star, triangle } = require('@jscad/modeling').primitives

// Main entry point; here we construct our solid:
const main = () => {

const thickness = 10;

const blank = translateZ(10/2, cylinder({ height: 10, radius: 10, segments: 64 }));
//return blank;

const profile = translateX(11, rotateZ(degToRad(55+180), triangle({type: 'SAS', values: [10, degToRad(110), 10]})));
//return profile;
const myshape = translateZ(5 + Math.sin(degToRad(55)) * 10, extrudeRotate({segments: 64}, profile));
//return myshape;

 
 // const boss = translateZ((thickness+5) / 2, cylinder({ height: thickness + 3, radius: 10, segments: 64 }))
  let gear = blank;//union(blank ,boss);
  

  gear = subtract(gear, myshape);
  const centerHole = translateZ(50/2, cylinder({ height: 50, radius: 3.2, segments: 16 }))
  gear = subtract(gear, centerHole);
  
  const flat = translate([(4/2) + (2/2), 0, (thickness + 0)/2], cuboid({size: [2, 6, thickness]}));
  let gap1 = cylinder({radius: 4, height: 20});
  
  let final = union(flat, gear);
  /*
  final = subtract(final, translate([11, 0, 0], gap1));
  final = subtract(final, translate([-11, 0, 0], gap1));
  final = subtract(final, translate([0, 11, 0], gap1));
  final = subtract(final, translate([0, -11, 0], gap1));
  */
  return final;  
}

module.exports = { main}

