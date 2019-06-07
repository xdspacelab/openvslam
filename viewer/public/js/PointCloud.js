const CLOUD_SIZE = 5000;

class PointCloud {
    constructor() {

        this.cloudMaterial = new THREE.PointsMaterial({
            size: property.PointSize,
            vertexColors: true/*,color:"rgb(255,255,0)"*/
        });


        this.clouds = [];
        this.cloudGeometries = [];
        this.cloudGeometryCaches = [];

        this.pointColors = [];

        this.totalPointCnt = 0;  // number of drew point, increase only
        this.nValidPoint = 0;  // number of points in SLAM
        this.vertexIds = {};  // vertex(Viewer上の点)のIDのリスト，3D点のIDで参照可能
        this.discardedPool = [];  // points removed from SLAM are not removed in viewer, will be reused when new point added
        this.discardedPoolSize = 0; // size of discardedPool

        this.prevReferencePointIds = [];

        this.POOL_POINT_COORDS = [0, 0, -100000]; // position of pooled point
    }


    // private methods
    addPoint(id, x, y, z, r, g, b) {
        // calc point coordinate
        let vector = new THREE.Vector3();
        vector.x = x;
        vector.y = y;
        vector.z = z;

        if (this.discardedPoolSize > 0) {
            let vertexId = this.discardedPool.pop();
            this.vertexIds[id] = vertexId;
            this.discardedPoolSize--;

            let cloudIdx = vertexId / CLOUD_SIZE | 0;
            let pointIdxInCloud = vertexId % CLOUD_SIZE;

            // reset point coordinate
            if (this.cloudGeometries[cloudIdx].vertices[pointIdxInCloud] === undefined) {
                console.error("The point is not in cloud geometry.")
            }
            this.cloudGeometries[cloudIdx].vertices[pointIdxInCloud].x = vector.x;
            this.cloudGeometries[cloudIdx].vertices[pointIdxInCloud].y = vector.y;
            this.cloudGeometries[cloudIdx].vertices[pointIdxInCloud].z = vector.z;
            this.cloudGeometries[cloudIdx].colors[pointIdxInCloud] = new THREE.Color("rgb(" + r + "," + g + "," + b + ")");
            this.cloudGeometries[cloudIdx].verticesNeedUpdate = true;

        } else {

            this.vertexIds[id] = this.totalPointCnt;
            this.totalPointCnt++;

            let cloudIdx = this.totalPointCnt / CLOUD_SIZE | 0;
            // Create geometry if previous geometry is full.
            if (!this.cloudGeometries[cloudIdx]) {
                this.cloudGeometries[cloudIdx] = new THREE.Geometry();
            }
            // add point coordinate
            let vertex = new THREE.Vector3(vector.x, vector.y, vector.z);
            this.cloudGeometries[cloudIdx].vertices.push(vertex);
            this.cloudGeometries[cloudIdx].colors.push(new THREE.Color("rgb(" + r + "," + g + "," + b + ")"));


            pointUpdateFlag = true;
        }
    }

    changePointPos(id, x, y, z) {
        let idx = this.vertexIds[id];
        let cloudIdx = idx / CLOUD_SIZE | 0;
        let pointIdx = idx % CLOUD_SIZE;

        // calc point coordinate
        let vector = new THREE.Vector3();
        vector.x = x;
        vector.y = y;
        vector.z = z;

        if (this.cloudGeometries[cloudIdx].vertices[pointIdx] === undefined) {
            this.cloudGeometries[cloudIdx].vertices[pointIdx] = new THREE.Vector3();
        }
        this.cloudGeometries[cloudIdx].vertices[pointIdx].x = vector.x;
        this.cloudGeometries[cloudIdx].vertices[pointIdx].y = vector.y;
        this.cloudGeometries[cloudIdx].vertices[pointIdx].z = vector.z;
        this.cloudGeometries[cloudIdx].verticesNeedUpdate = true;
    }

    changePointColor(id, r, g, b) {
        let vertexId = this.vertexIds[id];
        if (vertexId === undefined) return;//reference_keyframeから解除されたと同時に削除されている可能性がある
        let cloudIdx = vertexId / CLOUD_SIZE | 0;
        let vertexIdxOnCloud = vertexId % CLOUD_SIZE;
        this.cloudGeometries[cloudIdx].colors[vertexIdxOnCloud] = new THREE.Color("rgb(" + r + "," + g + "," + b + ")");
        this.cloudGeometries[cloudIdx].colorsNeedUpdate = true;
    }


    // public methods
    updatePoint(id, x, y, z, r, g, b) {

        if (this.vertexIds[id] === undefined || this.vertexIds[id] < 0) {
            this.addPoint(id, x, y, z, r, g, b);
            this.nValidPoint++;
        } else {
            this.changePointPos(id, x, y, z);
            this.changePointColor(id, r, g, b);
        }
        this.pointColors[id] = [r, g, b];
    }

    removePoint(id) {
        if (!(id in this.vertexIds)) {
            return;
        }
        // Move point to pool(origin)
        this.changePointPos(id, this.POOL_POINT_COORDS[0], this.POOL_POINT_COORDS[1], this.POOL_POINT_COORDS[2]);
        let vertexIdx = this.vertexIds[id];
        // Do nothing if point has been already removed.
        if (vertexIdx < 0) {
            return;
        }
        this.vertexIds[id] = -1;
        this.discardedPool.push(vertexIdx);
        this.discardedPoolSize++;
        this.nValidPoint--;
    }

    colorizeReferencePoints(referencePointIds) {

        for (let id of referencePointIds) {
            pointCloud.changePointColor(id, REFERENCE_POINT_COLOR[0], REFERENCE_POINT_COLOR[1], REFERENCE_POINT_COLOR[2]);
            this.prevReferencePointIds.splice(id, 1);

        }
        for (let id of this.prevReferencePointIds) {
            let color = this.pointColors[id];
            if (color !== undefined) {
                pointCloud.changePointColor(id, color[0], color[1], color[2]);
            }
        }
        this.prevReferencePointIds = referencePointIds;
    }

    updatePointInScene(scene) {
        for (let i = 0; i < this.cloudGeometries.length; i++) {

            if (this.clouds[i]) {
                scene.remove(this.clouds[i]);
                this.cloudGeometryCaches[i].dispose();
            }

            try {
                this.cloudGeometryCaches[i] = this.cloudGeometries[i].clone();
                this.cloudGeometryCaches[i].colors = this.cloudGeometries[i].colors; // should use concat?
                this.clouds[i] = new THREE.Points(this.cloudGeometryCaches[i], this.cloudMaterial);
                scene.add(this.clouds[i]);
            } catch (e) {
                console.error("error while cloning CloudGeometry: " + e);
                console.error(i + ", " + this.cloudGeometries[i]);
            }
        }
    }

    setPointSize(val) {
        this.cloudMaterial.size = val
    }
}
