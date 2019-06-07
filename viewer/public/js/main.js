'use strict';
const GLOBAL_SCALE = 50;

let scene, camera, renderer;


/* Color setting*/
const CURRENT_FRAME_COLOR = "rgb(0,192,0)";
const KEYFRAME_COLOR = "rgb(92, 85, 250)";
const EDGE_COLOR = "rgb(192, 223, 255)";
const BACKGROUND_COLOR = "rgb(255, 255, 255)";
const REFERENCE_POINT_COLOR = [255, 0, 0];

// timestamp on received for measuring fps;
let receiveTimestamp = 0;

let property = {
    CameraMode: 'Follow',
    FixAngle: true,
    LandmarkSize: 0.6,
    KeyframeSize: 0.5,
    CurrentFrameSize: 1.0,
    DrawGraph: true,
    DrawGrid: true,
    LocalizationMode: false,
    ResetSignal: function () { },
    StopSignal: function () { }
};

let graphicStats; // visualize fps of graphic refresh
let trackStats; // visualize fps of tracking update
let clock = new THREE.Clock();

let cameraFrames = new CameraFrames();

let pointUpdateFlag = false;
let pointCloud = new PointCloud();

let grid;

let mouseHandler;
let wheelHandler;
let viewControls;


function init() {

    // create a stats for showing graphic update rate
    // place on the left-up corner
    graphicStats = new Stats();
    graphicStats.setMode(0); // 0: fps, 1: ms
    graphicStats.domElement.style.position = "absolute";
    graphicStats.domElement.style.left = "0px";
    graphicStats.domElement.style.top = "0px";
    document.getElementById("Stats-output").appendChild(graphicStats.domElement);

    // create a stats for showing current frame update rate
    // place bellow of graphicStats
    trackStats = new Stats();
    trackStats.setMode(0);
    trackStats.domElement.style.position = "alsolute";
    trackStats.domElement.style.left = "0px";
    trackStats.domElement.style.top = "48px";
    document.getElementById("Stats-output").appendChild(trackStats.domElement);

    initGui();
    initTumbnail();
    initProtobuf();

    // create a scene, that holds all elements such as cameras and points.
    scene = new THREE.Scene();

    // create a camera
    camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.01, 1000);

    // create a render and set the setSize
    renderer = new THREE.WebGLRenderer({ antialias: false });
    renderer.setClearColor(new THREE.Color(BACKGROUND_COLOR));
    renderer.setSize(window.innerWidth, window.innerHeight);

    // create grid plane
    grid = new THREE.GridHelper(500, 50);
    scene.add(grid);

    // position and point the camera to the center of the scene
    camera.position.x = -100;
    camera.position.y = 60;
    camera.position.z = 30;
    camera.rotation.x = 0;
    camera.rotation.y = 0;
    camera.rotation.z = 0;

    let lineGeo = new THREE.Geometry();
    for (let i = 0; i < 16; i++) {
        lineGeo.vertices.push(new THREE.Vector3(0, 0, 0));
    }

    // add the output of the renderer to the html element
    // this line must be before initialization TrackBallControls(othrewise, dat.gui won't be work).
    document.getElementById("WebGL-output").appendChild(renderer.domElement);

    // create a view controller that
    viewControls = new ViewControls(camera);

    // create a mouse action listener
    let mouseListener = function (btn, act, pos, vel) {
        if (btn == 0 && act != 0) {
            viewControls.addRot(vel[0], vel[1]);
        }
        else if (btn == 2 && act != 0) {
            viewControls.addMove(vel[0], vel[1])
        }
    };
    // create a mouse wheel action listener
    let wheelListener = function (rot) {
        viewControls.addZoom(rot);
    };
    mouseHandler = new MouseHandler(renderer.domElement, mouseListener);
    wheelHandler = new WheelHandler(renderer.domElement, wheelListener);
    setCameraMode(property.CameraMode);
    viewControls.update(100);

    // animation render function
    render();

}

// render method that updates each stats, camera frames, view controller, and renderer.
function render() {
    graphicStats.update();

    pointCloud.updatePointInScene(scene);

    cameraFrames.updateFramesInScene(scene);

    //if(chase_camera == false){
    // 仮　トラックボールコントロール用
    let delta = clock.getDelta();
    //trackballControls.update(delta);
    viewControls.update(delta);


    // render using requestAnimationFrame
    requestAnimationFrame(render);
    // render the Scene
    renderer.render(scene, camera);
}

// initialize gui by dat.gui
function initGui() {
    let gui = new dat.GUI({ width: 300 });

    gui.add(property, 'CameraMode', ['Above', 'Follow', 'Bird', 'Subjective']).onChange(setCameraMode);
    gui.add(property, 'FixAngle').onChange(toggleFixAngle);
    gui.add(property, 'LandmarkSize', 0, 4, 0.1).onChange(setPointSize);
    gui.add(property, 'KeyframeSize', 0, 4, 0.1).onChange(setKeyframeSize);
    gui.add(property, 'CurrentFrameSize', 0, 4, 0.1).onChange(setCurrentframeSize);
    gui.add(property, 'DrawGraph').onChange(setGraphVis);
    gui.add(property, 'DrawGrid').onChange(setGridVis);
    gui.add(property, 'LocalizationMode').onChange(setLocalizationMode);
    gui.add(property, 'ResetSignal').domElement.children[0].innerHTML = "<button onclick='onClickReset()'>reset</button>";
    gui.add(property, 'StopSignal').domElement.children[0].innerHTML = "<button onclick='onClickTerminate()'>terminate</button>";
}

function setCameraMode(val) {
    let suffix = "_rot";
    if (property.FixAngle) {
        suffix = "_fix";
    }
    cameraFrames.setCurrentFrameVisibility(val !== "Subjective");
    viewControls.setMode(val + suffix);
}
function toggleFixAngle(val) {
    // If view angle is fixed, camera could not be rotated.
    setCameraMode(property.CameraMode);
}
function setPointSize(val) {
    val = Math.pow(2, val);
    pointCloud.setPointSize(val);
}
function setKeyframeSize(val) {
    val = Math.pow(2, val);
    cameraFrames.setKeyframeSize(val);
}
function setCurrentframeSize(val) {
    val = Math.pow(2, val);
    cameraFrames.setCurrentFrameSize(val);
}
function setGraphVis(val) {
    cameraFrames.setGraphVisibility(val);
}
function setGridVis(val) {
    grid.visible = val;
}
function setLocalizationMode(val) {
    if (val == true) {
        socket.emit("signal", "disable_mapping_mode");
    }
    else {
        socket.emit("signal", "enable_mapping_mode");
    }
}
function onClickReset() {
    socket.emit("signal", "reset");
}
function onClickTerminate() {
    socket.emit("signal", "terminate");
}

// function that converts array that have size of 16 to matrix that shape of 4x4
function array2mat44(mat, array) {
    for (let i = 0; i < 4; i++) {
        let raw = [];
        for (let j = 0; j < 4; j++) {
            let k = i * 4 + j;
            let elm = array[k];
            raw.push(elm);
        }
        mat.push(raw);
    }
}
function loadProtobufData(obj, keyframes, edges, points, referencePointIds, currentFramePose) {
    for (let keyframeObj of obj.keyframes) {
        let keyframe = {};
        keyframe["id"] = keyframeObj.id;
        if (keyframeObj.pose != undefined) {
            keyframe["camera_pose"] = [];
            array2mat44(keyframe["camera_pose"], keyframeObj.pose.pose);
        }
        keyframes.push(keyframe);
    }
    for (let edgeObj of obj.edges) {
        edges.push([edgeObj.id0, edgeObj.id1])
    }
    for (let landmarkObj of obj.landmarks) {
        let landmark = {};
        landmark["id"] = landmarkObj.id;
        if (landmarkObj.coords.length != 0) {
            landmark["point_pos"] = landmarkObj.coords;
            landmark["rgb"] = landmarkObj.color;
        }
        points.push(landmark);
    }
    for (let id of obj.localLandmarks) {
        referencePointIds.push(id);
    }
    array2mat44(currentFramePose, obj.currentFrame.pose);

}

let mapSegment = undefined;
let mapMsg = undefined;
function initProtobuf() {

    protobuf.load("map_segment.proto", function (err, root) {
        mapSegment = root.lookupType("map_segment.map");
        mapMsg = root.lookupType("map_segment.map.msg");
    });
}

function receiveProtobuf(msg) {
    if (msg.length == 0 || mapSegment == undefined) {
        return;
    }

    let keyframes = [];
    let edges = [];
    let points = [];
    let referencePointIds = [];
    let currentFramePose = [];

    let buffer = base64ToUint8Array(msg);
    let obj = mapSegment.decode(buffer);

    if (obj.messages[0].tag == "RESET_ALL") {
        removeAllElements();
    }
    else {
        loadProtobufData(obj, keyframes, edges, points, referencePointIds, currentFramePose);
        updateMapElements(msg.length, keyframes, edges, points, referencePointIds, currentFramePose);
    }
}
function base64ToUint8Array(base64) {
    let binaryString = window.atob(base64);
    let len = binaryString.length;
    let bytes = new Uint8Array(len);
    for (var i = 0; i < len; i++) {
        bytes[i] = binaryString.charCodeAt(i);
    }
    return bytes;
}

function updateMapElements(msgSize, keyframes, edges, points, referencePointIds, currentFramePose) {
    trackStats.update();
    cameraFrames.updateCurrentFrame(currentFramePose);
    viewControls.setCurrentIntrinsic(currentFramePose);

    if (cameraFrames.numValidKeyframe == 0 && keyframes.length == 0) {
        return;
    }

    for (let point of points) {
        let id = point["id"];
        if (point["point_pos"] == undefined) {
            pointCloud.removePoint(id);
        }
        else {
            let x = point["point_pos"][0] * GLOBAL_SCALE;
            let y = point["point_pos"][1] * GLOBAL_SCALE;
            let z = point["point_pos"][2] * GLOBAL_SCALE;
            let r = point["rgb"][0];
            let g = point["rgb"][1];
            let b = point["rgb"][2];
            pointCloud.updatePoint(id, x, y, z, r, g, b);
        }
    }
    for (let keyframe of keyframes) {
        let id = keyframe["id"];
        if (keyframe["camera_pose"] == undefined) {
            cameraFrames.removeKeyframe(id);
        }
        else {
            cameraFrames.updateKeyframe(id, keyframe["camera_pose"]);
        }
    }
    cameraFrames.setEdges(edges);


    let currentMillis = new Date().getTime();
    if (receiveTimestamp != 0) {
        let dt = currentMillis - receiveTimestamp;
        if (dt < 2) dt = 2;
        let fps = 1000.0 / dt;
        // adaptive update rate
        //viewControls.updateSmoothness(fps);
        console.log(("         " + parseInt(msgSize / 1000)).substr(-6) + " KB"
            + ("     " + (fps).toFixed(1)).substr(-7) + " fps, "
            + ("         " + pointCloud.nValidPoint).substr(-6) + " pts, "
            + ("         " + cameraFrames.numValidKeyframe).substr(-6) + " kfs");
    }
    receiveTimestamp = currentMillis;

    pointCloud.colorizeReferencePoints(referencePointIds);

}

function removeAllElements() {
    for (let id in pointCloud.vertexIds) {
        if (id < 0 || id == undefined) {
            continue;
        }
        pointCloud.removePoint(id);
    }
    for (let id in cameraFrames.keyframeIndices) {
        if (id < 0 || id == undefined) {
            continue;
        }
        cameraFrames.removeKeyframe(id);
    }
    cameraFrames.setEdges([]);
}

// calculate inverse of se3 pose matrix
function inv(pose) {
    let res = new Array();
    for (let i = 0; i < 3; i++) {
        res.push([0, 0, 0, 0]);
    }
    // - R^T * t
    res[0][3] = - pose[0][0] * pose[0][3] - pose[1][0] * pose[1][3] - pose[2][0] * pose[2][3];
    res[1][3] = - pose[0][1] * pose[0][3] - pose[1][1] * pose[1][3] - pose[2][1] * pose[2][3];
    res[2][3] = - pose[0][2] * pose[0][3] - pose[1][2] * pose[1][3] - pose[2][2] * pose[2][3];
    res[0][0] = pose[0][0]; res[0][1] = pose[1][0]; res[0][2] = pose[2][0];
    res[1][0] = pose[0][1]; res[1][1] = pose[1][1]; res[1][2] = pose[2][1];
    res[2][0] = pose[0][2]; res[2][1] = pose[1][2]; res[2][2] = pose[2][2];

    return res;
}

// window resize function
// The function is called in index.ejs
function onResize() {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
}

let thumbEnlarge = false; // if thumbnail is clicked, that is enlarged and this flag is set
const THUMB_SCALING = 3; // thumbnail scaling magnification
const THUMB_HEIGHT = 96; // normally thumbnail height (width is doubled height)
const CANVAS_SIZE = [1024, 500]; // thumbnail image resolution
function initTumbnail() {
    let thumb = document.getElementById("thumb");
    thumb.style.width = THUMB_HEIGHT * 2 + 'px';
    thumb.style.height = THUMB_HEIGHT + 'px';
    thumb.style.transition = 'all 0.5s ease-in-out'; // enable animation when enlarge and shrinking
    thumb.style.zIndex = '10001'; // thumbnail is drawn over two stats
    thumb.setAttribute("width", CANVAS_SIZE[0]);
    thumb.setAttribute("height", CANVAS_SIZE[1]);
    thumb.addEventListener('click', onThumbClick);

}
function onThumbClick() {

    thumbEnlarge = !thumbEnlarge; // inverse flag
    if (!thumbEnlarge) {
        document.getElementById("thumb").style.transform = 'translate(0px, 0px) scale(1)';
    }
    else {
        let x = THUMB_HEIGHT * (THUMB_SCALING - 1);
        let y = THUMB_HEIGHT / 2 * (THUMB_SCALING - 1);
        document.getElementById("thumb").style.transform = 'translate(' + x + 'px, ' + y + 'px) scale(' + THUMB_SCALING + ')';
    }

}