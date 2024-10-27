
"use strict";

let PosTrackStatus = require('./PosTrackStatus.js');
let Skeleton3D = require('./Skeleton3D.js');
let PlaneStamped = require('./PlaneStamped.js');
let Keypoint2Df = require('./Keypoint2Df.js');
let Keypoint3D = require('./Keypoint3D.js');
let Keypoint2Di = require('./Keypoint2Di.js');
let ObjectsStamped = require('./ObjectsStamped.js');
let BoundingBox3D = require('./BoundingBox3D.js');
let BoundingBox2Df = require('./BoundingBox2Df.js');
let RGBDSensors = require('./RGBDSensors.js');
let Object = require('./Object.js');
let BoundingBox2Di = require('./BoundingBox2Di.js');
let Skeleton2D = require('./Skeleton2D.js');

module.exports = {
  PosTrackStatus: PosTrackStatus,
  Skeleton3D: Skeleton3D,
  PlaneStamped: PlaneStamped,
  Keypoint2Df: Keypoint2Df,
  Keypoint3D: Keypoint3D,
  Keypoint2Di: Keypoint2Di,
  ObjectsStamped: ObjectsStamped,
  BoundingBox3D: BoundingBox3D,
  BoundingBox2Df: BoundingBox2Df,
  RGBDSensors: RGBDSensors,
  Object: Object,
  BoundingBox2Di: BoundingBox2Di,
  Skeleton2D: Skeleton2D,
};
