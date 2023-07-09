
    


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

  const thickness = 0.4 * 5;
  let blankOutline = polygon({points: [
    [-25, 0],
    [-30, -05],
    [-30, -35],
    
    [-10, -40],
    [0, -42],
    [10, -40],
    
    [30, -35],
    [30, -05],
    [25, 0],
    ]})
    
  let blank = extrudeLinear({height: thickness}, blankOutline);
  blank = rotateZ(degToRad(90), blank);
  
  let base = cuboid({size: [60, 40, thickness]});
  base = translate([60, 0, thickness/2], base);
  blank = union(blank, base);
  
  let bHole = cylinder({radius: 2.2, height: 30});
  blank = subtract(blank, translate([5, 25, 0], bHole));
  blank = subtract(blank, translate([5, -25, 0], bHole));
  blank = subtract(blank, translate([35, 0, 0], bHole));
  
  let penHole = cylinder({radius: 14/2, height: 30});
  blank = subtract(blank, penHole);
  
  let trim2 = cuboid({size: [20, 40, 10]});
  blank = subtract(blank, translate([20, 0, 0], trim2));
 
  let trims = cuboid({size: [14, 10, 10]});
  blank = subtract(blank, translate([20, -10, 0], trims));
  
  let batslot = cuboid({size: [2.5, 40, 8]});
  let bs1 = translate([45, 0, 8/2], batslot);
  let bs2 = translate([85, 0, 8/2], batslot);
  
  let bclip = cuboid({size: [8, 78, thickness]})
  let bclipt = cuboid({size: [14, 3, 8]})
  bclip = union(bclip, translate([0, 35 + 3, 4 - (thickness/2)], bclipt));
  bclip = union(bclip, translate([0, -35 - 3, 4 - (thickness/2)], bclipt));
  
  blank = union(blank, translate([65, 0, thickness/2], bclip))
  
  let trim3 = cuboid({size: [30, 20, 10]});
  trim3 = translate([65, 0, 0], trim3);
  
  blank = union(blank, bs1, bs2);
  blank = subtract(blank, trim3);
  
  let trim4 = cuboid({size: [50, 20, 10]});
  trim4 = translate([65, 0, 10/2 + thickness], trim4);
  blank = subtract(blank, trim4);
  return [blank];
 
}

module.exports = { main}
