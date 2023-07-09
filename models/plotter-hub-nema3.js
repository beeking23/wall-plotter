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

  const thickness = 0.4 * 6;

  let motor = cylinder({radius: 23/2, height: 20});
 
  let blankOutline = polygon({points: [
    [35, 45], 
    [-40, 45],
    [-51, 55],
    [-62, 42],
    [-55, 05],
    [-50, 0],
    [-35, 0],
    [-30, -05],
    [-30, -35],
    
    [-10, -35],
    [0, -42],
    [10, -35],
    
    [30, -35],
    [30, -05],
    [35, 0],
    [50, 0],
    [55, 05],
    [62, 42],
    [51, 55],
    [40, 45]]})
  let blank = extrudeLinear({height: thickness}, blankOutline);
  
  blank = rotateZ(degToRad(90), blank);
    
  blank = subtract(blank, translate([-23, 30, 0], motor));
  blank = subtract(blank, translate([-23, -30, 0], motor));
  
  let mcHole = cylinder({radius: 2, height: 30});
  const f = 23/2 + 4
  blank = subtract(blank, translate([-23 - f, 30 - f, 0], mcHole));
  blank = subtract(blank, translate([-23 + f, 30 + f, 0], mcHole));
  
  blank = subtract(blank, translate([-23 + f, -30 - f, 0], mcHole));
  blank = subtract(blank, translate([-23 - f, -30 + f, 0], mcHole));
  
  let penHole = cylinder({radius: 9.5/2, height: 30});
  blank = subtract(blank, penHole);
 
  let trim2 = cuboid({size: [20, 20, 10]});
  blank = subtract(blank, translate([20, 0, 0], trim2));
 
  let trim3 = cuboid({size: [20, 20, 10]});
  blank = subtract(blank, translate([-25, 0, 0], trim3));
  
  let trims = cuboid({size: [14, 10, 10]});
  blank = subtract(blank, translate([20, -10, 0], trims));
 
  let feed = cuboid({size: [4, 10, 7], center: [0, 0, 7/2 + thickness]});
  blank = union(blank, translate([-43, 0, 0], feed));
  
 
 
  
  let svr1 = cuboid({size: [2, 7, 6], center: [0, 0, 4/2 + thickness]});
  blank = union(blank, translate([10, -20, 0], svr1));
  blank = union(blank, translate([30, -20, 0], svr1));
  
  let bHole = cylinder({radius: 2.2, height: 30});
  blank = subtract(blank, translate([5, 25, 0], bHole));
  blank = subtract(blank, translate([5, -25, 0], bHole));
  blank = subtract(blank, translate([35, 0, 0], bHole));
  
  return [ blank];
 
}

module.exports = { main}
