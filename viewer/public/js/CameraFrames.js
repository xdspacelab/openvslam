class CameraFrames {

    constructor() {

        /* Camera componet */
        this.CURRENT_FRAME_MATERIAL = new THREE.LineBasicMaterial({ color: CURRENT_FRAME_COLOR });
        this.KEYFRAME_MATERIAL = new THREE.LineBasicMaterial({ color: KEYFRAME_COLOR });
        this.EDGE_MATERIAL = new THREE.LineBasicMaterial({ color: EDGE_COLOR });

        this.keyframeIndices = [];
        this.keyframeObjects = [];
        this.keyframePoses = [];
        this.addedKeyframeIndices = [];
        this.removedPool = [];
        this.removedPoolSize = 0;

        this.totalFrameCount = 0;
        this.numValidKeyframe = 0;

        this.POOL_KEYFRAME_POSE = [[1, 0, 0, 0], [0, 1, 0, -100000], [0, 0, 1, 0]];

        let lineaGeometry = this.makeWireframe(this.POOL_KEYFRAME_POSE, this.currentFrameWireSize);
        this.currentFrame = new THREE.LineSegments(lineaGeometry, this.CURRENT_FRAME_MATERIAL);
        this.flagCurrentFrameInScene = false;
        this.currentFramePose = this.POOL_KEYFRAME_POSE;

        this.currentFrameWireSize = Math.pow(2, property.CurrentFrameSize);
        this.keyframeWireSize = Math.pow(2, property.KeyframeSize);

        lineaGeometry = new THREE.Geometry();
        this.edges = new THREE.LineSegments(lineaGeometry, this.EDGE_MATERIAL);
        this.flagEdgesInScene = false;
    }

    // private methods

    addKeyframe(id, pose) {

        if (this.removedPoolSize > 0) {
            let index = this.removedPool.pop();
            this.removedPoolSize--;

            this.keyframeIndices[id] = index;
            this.changeKeyframePos(index, pose);
        }
        else {
            let lineGeometry = this.makeWireframe(pose, this.keyframeWireSize);
            let keyframeObject = new THREE.LineSegments(lineGeometry, this.KEYFRAME_MATERIAL);

            this.keyframeIndices[id] = this.totalFrameCount;
            this.addedKeyframeIndices.push(this.totalFrameCount);
            this.keyframeObjects[this.totalFrameCount] = keyframeObject;
            this.keyframePoses[this.totalFrameCount] = pose;

            this.totalFrameCount++;
        }

        this.numValidKeyframe++;
    }

    removeKeyframe(id) {

        let index = this.keyframeIndices[id];

        if (this.keyframeIndices[id] < 0 || index === undefined)
            return;

        this.changeKeyframePos(index, this.POOL_KEYFRAME_POSE);

        this.keyframeIndices[id] = -1;
        this.removedPool.push(index);
        this.removedPoolSize++;

        this.numValidKeyframe--;
    }

    changeKeyframePos(index, pose) {

        let lineGeometry = this.makeWireframe(pose, this.keyframeWireSize);

        this.keyframeObjects[index].geometry = lineGeometry;
        this.keyframeObjects[index].geometry.verticesNeedUpdate = true;
        this.keyframePoses[index] = pose;
    }

    makeWireframe(pose_, size) {
        let lineGeo = new THREE.Geometry();
        const width = 0.08 / 2 * GLOBAL_SCALE * size;
        const height = 0.045 / 2 * GLOBAL_SCALE * size;
        const depth = 0.02 * GLOBAL_SCALE * size;
        let pose = inv(pose_);
        let Ox = pose[0][3] * GLOBAL_SCALE; let Oy = pose[1][3] * GLOBAL_SCALE; let Oz = pose[2][3] * GLOBAL_SCALE;
        let e1x = width * pose[0][0]; let e1y = width * pose[1][0]; let e1z = width * pose[2][0]; // local x-axis on world
        let e2x = height * pose[0][1]; let e2y = height * pose[1][1]; let e2z = height * pose[2][1]; // local y-axis on world
        let e3x = depth * pose[0][2]; let e3y = depth * pose[1][2]; let e3z = depth * pose[2][2]; // local z-axis on world

        // center to 4 vertices
        lineGeo.vertices.push(new THREE.Vector3(Ox, Oy, Oz));
        lineGeo.vertices.push(new THREE.Vector3(Ox + e1x + e2x + e3x, Oy + e1y + e2y + e3y, Oz + e1z + e2z + e3z));
        lineGeo.vertices.push(new THREE.Vector3(Ox, Oy, Oz));
        lineGeo.vertices.push(new THREE.Vector3(Ox + e1x - e2x + e3x, Oy + e1y - e2y + e3y, Oz + e1z - e2z + e3z));
        lineGeo.vertices.push(new THREE.Vector3(Ox, Oy, Oz));
        lineGeo.vertices.push(new THREE.Vector3(Ox - e1x - e2x + e3x, Oy - e1y - e2y + e3y, Oz - e1z - e2z + e3z));
        lineGeo.vertices.push(new THREE.Vector3(Ox, Oy, Oz));
        lineGeo.vertices.push(new THREE.Vector3(Ox - e1x + e2x + e3x, Oy - e1y + e2y + e3y, Oz - e1z + e2z + e3z));

        // among 4 vertices
        lineGeo.vertices.push(new THREE.Vector3(Ox + e1x + e2x + e3x, Oy + e1y + e2y + e3y, Oz + e1z + e2z + e3z));
        lineGeo.vertices.push(new THREE.Vector3(Ox + e1x - e2x + e3x, Oy + e1y - e2y + e3y, Oz + e1z - e2z + e3z));
        lineGeo.vertices.push(new THREE.Vector3(Ox + e1x - e2x + e3x, Oy + e1y - e2y + e3y, Oz + e1z - e2z + e3z));
        lineGeo.vertices.push(new THREE.Vector3(Ox - e1x - e2x + e3x, Oy - e1y - e2y + e3y, Oz - e1z - e2z + e3z));
        lineGeo.vertices.push(new THREE.Vector3(Ox - e1x - e2x + e3x, Oy - e1y - e2y + e3y, Oz - e1z - e2z + e3z));
        lineGeo.vertices.push(new THREE.Vector3(Ox - e1x + e2x + e3x, Oy - e1y + e2y + e3y, Oz - e1z + e2z + e3z));
        lineGeo.vertices.push(new THREE.Vector3(Ox - e1x + e2x + e3x, Oy - e1y + e2y + e3y, Oz - e1z + e2z + e3z));
        lineGeo.vertices.push(new THREE.Vector3(Ox + e1x + e2x + e3x, Oy + e1y + e2y + e3y, Oz + e1z + e2z + e3z));

        return lineGeo;
    }

    // public methods

    updateKeyframe(id, pose) {
        let index = this.keyframeIndices[id];

        if (index < 0 || index === undefined) {
            this.addKeyframe(id, pose);
        }
        else {
            this.changeKeyframePos(index, pose);
        }
    }

    updateCurrentFrame(pose) {
        let lineGeometry = this.makeWireframe(pose, this.currentFrameWireSize);
        this.currentFrame.geometry = lineGeometry;
        this.currentFrame.geometry.verticesNeedUpdate = true;
        this.currentFramePose = pose;
    }


    setKeyframeSize(val) {
        this.keyframeWireSize = val;
        for (let index of this.keyframeIndices) {
            if (index < 0 || index == undefined) {
                continue;
            }
            this.changeKeyframePos(index, this.keyframePoses[index]);
        }
    }

    setCurrentFrameSize(val) {
        this.currentFrameWireSize = val;
        this.currentFrame.geometry = this.makeWireframe(this.currentFramePose, val);
    }

    setCurrentFrameVisibility(visibility) {
        this.currentFrame.visible = visibility;
    }

    updateFramesInScene(scene) {

        if (!this.flagEdgesInScene) {
            this.flagEdgesInScene = true;
            scene.add(this.edges);
        }

        for (let index in this.addedKeyframeIndices) {
            scene.add(this.keyframeObjects[index]);
        }

        if (!this.flagCurrentFrameInScene) {
            this.flagCurrentFrameInScene = true;
            scene.add(this.currentFrame);
        }
    }

    setEdges(edges) {
        let lineGeometry = new THREE.Geometry();
        for (let edge of edges) {
            let id0 = this.keyframeIndices[edge[0]];
            let id1 = this.keyframeIndices[edge[1]];
            if (id0 == undefined || id1 == undefined) {
                continue;
            }
            let pose0 = inv(this.keyframePoses[id0]);
            let pose1 = inv(this.keyframePoses[id1]);

            let x1 = pose0[0][3] * GLOBAL_SCALE; let y1 = pose0[1][3] * GLOBAL_SCALE; let z1 = pose0[2][3] * GLOBAL_SCALE;
            let x2 = pose1[0][3] * GLOBAL_SCALE; let y2 = pose1[1][3] * GLOBAL_SCALE; let z2 = pose1[2][3] * GLOBAL_SCALE;

            lineGeometry.vertices.push(new THREE.Vector3(x1, y1, z1));
            lineGeometry.vertices.push(new THREE.Vector3(x2, y2, z2));
        }
        let oldLineGeometry = this.edges.geometry;
        this.edges.geometry = lineGeometry;
        oldLineGeometry.dispose();
    }
    setGraphVisibility(visible) {
        this.edges.visible = visible;
    }

}
