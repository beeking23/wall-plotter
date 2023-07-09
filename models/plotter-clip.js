 const jscad = require('@jscad/modeling')
const { cylinder, cylinderElliptic, polygon, cuboid } = jscad.primitives
const { rotateZ, translateZ, translateY, rotateX,rotateY, translateX, translate } = jscad.transforms
const { extrudeLinear, extrudeRotate } = jscad.extrusions
const { union, subtract } = jscad.booleans
const { vec2 } = jscad.maths
const { degToRad } = jscad.utils
const { cube, ellipse, star, triangle, circle, rectangle } = require('@jscad/modeling').primitives
const { expand, offset } = require('@jscad/modeling').expansions
const { hull, hullChain } = require('@jscad/modeling').hulls

// Main entry point; here we construct our solid:
const main = () => {

const thickness = 8;


const c = circle({center: [0, 0], radius: 11.5/2});
const r = rectangle({center: [0, 2], size: [6, 16]});
const t =  translate([-3, 10, 0], triangle({type: 'SSS', values: [6, 6, 6]}));

const s = rectangle({center: [0, -5], size: [4, 10]});

const h = rectangle({center: [5, 10], size: [10, 10]});

const f = union(c, r, t);


const d1 = extrudeLinear({height: thickness}, expand({delta: -1.5, corners: 'edge'}, f));

let d2 = extrudeLinear({height: thickness}, f);
d2 = union(d2, extrudeLinear({height: 1.2}, h));


const d3 = subtract(d2, d1);

const clip = subtract(d3, extrudeLinear({height: thickness}, s));

const bev = translateZ(8, cylinderElliptic({height: 3, startRadius: [8.5/2, 8.5/2], endRadius: [11/2, 11/2]}));

return [h, subtract(clip, bev)];
  
}

module.exports = { main}

    
