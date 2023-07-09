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

const blank = translateZ(10/2, cylinder({ height: 10, radius: 15, segments: 64 }));
//return blank;

const profile = translateX(16, rotateZ(degToRad(55+180), triangle({type: 'SAS', values: [10, degToRad(110), 10]})));
//return profile;
const myshape = translateZ(5 + Math.sin(degToRad(55)) * 10, extrudeRotate({segments: 64}, profile));
//return myshape;
let gear = subtract(blank, myshape);
 
  const boss = translateZ((thickness+5) / 2, cylinder({ height: thickness + 5, radius: 5, segments: 64 }))
  gear = union(gear,boss);
  
  const centerHole = translateZ(50/2, cylinder({ height: 50, radius: 3.1, segments: 16 }))
  gear = subtract(gear, centerHole);
  
  const flat = translate([(3.9/2) + (2/2), 0, (thickness + 5)/2], cuboid({size: [2, 5, thickness + 5]}));
  
  return union(flat, gear);
  
  
}

module.exports = { main}

