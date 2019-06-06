class ControlMethod {
    constructor(name) {
        this.name = name;

        // used for reference observer pose.
        this.vp = new THREE.Vector3(0, 0, 0);  // observer view point
        this.th = 0;
        this.ph = -Math.PI / 2;
        this.ps = 0;
        this.dist = 1 * GLOBAL_SCALE;

        this.baseTh = 0;
        this.basePh = 0;

        // current frame variables
        this.frameEye = new THREE.Vector3(1, 0, 0);
        this.frameCenter = new THREE.Vector3();

        // update call backs
        this.updateCallback = undefined;

        this.referenceRot = new THREE.Vector3();
        this.referenceVp = new THREE.Vector3();
        this.referenceDist = 20;

        // constants
        this.SENSI_ROT = 0.01; // Sensitivity of rotation.
        this.SENSI_MOVE = 0.4; // Sensitivity of translation
        this.SENSI_ZOOM = 180; // Sensitivity of Zooming.

    }
    setUpdateCallback(func) {
        this.updateCallback = func;
    }
    setInitialRot(th, ph, ps) {
        this.th = th; this.ph = ph; this.ps = ps;
    }

    addMove(dx, dy) {
        this.updateCallback(-dx * this.SENSI_MOVE, -dy * this.SENSI_MOVE, 0, 0);
    }
    addRot(dx, dy) {
        this.updateCallback(0, 0, dx * this.SENSI_ROT, -dy * this.SENSI_ROT);
    }
    addZoom(q) {
        this.dist += this.SENSI_ZOOM * q;
        if (this.dist < 1) {
            this.dist = 1;
        }
    }

    computeReferences() {

        this.referenceRot = new THREE.Vector3(this.ph + this.basePh + Math.PI, this.th + this.baseTh, this.ps);
        this.referenceVp.copy(this.vp);
        this.referenceDist = this.dist;

    }
    getReferenceRot() {
        return this.referenceRot;
    }
    getReferenceVp() {
        return this.referenceVp;
    }
    getReferenceDist() {
        return this.referenceDist;
    }

    updateReferences() {
        this.updateCallback(0, 0, 0, 0);
    }
}

class ViewControls {

    constructor(camera, initCamreaMode) {

        this.camera = camera;
        this.camera.rotation.order = "YXZ";

        // current status
        this.currentRot = new THREE.Vector3();
        this.currentVp = new THREE.Vector3();
        this.currentDist = 20;

        this.tau = 0.15; // Time parameter, half-life.

        this.allMethod = [];

        let aboveFix = new ControlMethod("Above_fix");
        aboveFix.setUpdateCallback(function (dx, dy, th, ph) {
            this.vp.copy(this.frameCenter);

            let x = this.frameEye.x;
            let z = this.frameEye.z;

            this.baseTh = Math.atan2(x, z);
            this.basePh = -Math.PI / 2;
            this.th = 0;
            this.ph = 0;
            this.ps = 0;
        });
        this.allMethod.push(aboveFix);

        let aboveRot = new ControlMethod("Above_rot");
        aboveRot.setUpdateCallback(function (dx, dy, th, ph) {
            this.vp.copy(this.frameCenter);

            let x = this.frameEye.x;
            let z = this.frameEye.z;

            this.baseTh = Math.atan2(x, z);
            this.basePh = -Math.PI / 2;
            this.th += th;
            this.ph = 0;
            this.ps = 0;
        });
        this.allMethod.push(aboveRot);

        const angle = Math.PI / 4;
        let followFix = new ControlMethod("Follow_fix");
        followFix.setInitialRot(0, -Math.PI / 2 + angle, 0);
        followFix.setUpdateCallback(function (dx, dy, th, ph) {
            this.vp.copy(this.frameCenter);

            let x = this.frameEye.x;
            let z = this.frameEye.z;
            this.baseTh = Math.atan2(x, z);
            this.basePh = -Math.PI / 2 + angle;
            this.th = 0;
            this.ph = 0;
            this.ps = 0;

        });
        this.allMethod.push(followFix);

        let followRot = new ControlMethod("Follow_rot");
        followRot.setInitialRot(0, -Math.PI / 2 + angle, 0);
        followRot.setUpdateCallback(function (dx, dy, th, ph) {
            this.vp.copy(this.frameCenter);

            let x = this.frameEye.x;
            let z = this.frameEye.z;
            this.baseTh = Math.atan2(x, z);
            this.basePh = -Math.PI / 2 + angle;

            this.th += th;
            this.ph += ph;
            this.ps = 0;

        });
        this.allMethod.push(followRot);

        let birdRot = new ControlMethod("Bird_rot");
        birdRot.setUpdateCallback(function (dx, dy, th, ph) {
            // x axis of local coordinate on world coordinate
            let ex_x = Math.cos(this.th);
            let ex_y = 0;
            let ex_z = -Math.sin(this.th);
            // y axis of local coordinate on world coordinate
            let ey_x = Math.sin(this.ph) * Math.sin(this.th);
            let ey_y = Math.cos(this.ph);
            let ey_z = Math.sin(this.ph) * Math.cos(this.th);

            this.vp.x += ex_x * dx + ey_x * dy;
            this.vp.y += ex_y * dx + ey_y * dy;
            this.vp.z += ex_z * dx + ey_z * dy;

            this.baseTh = 0;
            this.basePh = 0;

            this.th += th;
            this.ph += ph;

        });
        this.allMethod.push(birdRot);

        let birdFix = new ControlMethod("Bird_fix");
        birdFix.setUpdateCallback(birdRot.updateCallback);
        this.allMethod.push(birdFix);

        let subjectiveFix = new ControlMethod("Subjective_fix");
        subjectiveFix.setUpdateCallback(function (dx, dy, th, ph) {
            this.vp.copy(this.frameCenter);

            let x = this.frameEye.x;
            let y = this.frameEye.y;
            let z = this.frameEye.z;

            this.baseTh = Math.atan2(x, z);
            this.basePh = -Math.atan2(y, Math.sqrt(x * x + z * z));

            this.th = 0;
            this.ph = 0;
            this.ps = 0;

            this.dist = 0;

        });
        this.allMethod.push(subjectiveFix);

        let subjectiveRot = new ControlMethod("Subjective_rot");
        subjectiveRot.setUpdateCallback(function (dx, dy, th, ph) {
            this.vp.copy(this.frameCenter);

            let x = this.frameEye.x;
            let y = this.frameEye.y;
            let z = this.frameEye.z;

            this.baseTh = Math.atan2(x, z);
            this.basePh = -Math.atan2(y, Math.sqrt(x * x + z * z));

            this.th -= th; // 主観表示では回転は逆のほうが自然
            this.ph -= ph;
            this.dist = 0;

        });
        this.allMethod.push(subjectiveRot);

        this.currentMethod = this.allMethod[0];
        this.setMode(initCamreaMode);

    }

    setCurrentIntrinsic(pose) {
        let eye = new THREE.Vector3(pose[2][0], pose[2][1], pose[2][2]);
        let ocx = -pose[0][0] * pose[0][3] - pose[1][0] * pose[1][3] - pose[2][0] * pose[2][3];
        let ocy = -pose[0][1] * pose[0][3] - pose[1][1] * pose[1][3] - pose[2][1] * pose[2][3];
        let ocz = -pose[0][2] * pose[0][3] - pose[1][2] * pose[1][3] - pose[2][2] * pose[2][3];
        let center = new THREE.Vector3(ocx * GLOBAL_SCALE, ocy * GLOBAL_SCALE, ocz * GLOBAL_SCALE);

        for (let method of this.allMethod) {
            method.frameEye = eye;
            method.frameCenter = center;
            method.updateReferences();
        }
    }

    setMode(mode) {
        this.currentMethod = this.allMethod[0];
        for (let method of this.allMethod) {
            if (method.name == mode) {
                this.currentMethod = method;
                break;
            }
        }
    }

    addRot(dx, dy) {
        this.currentMethod.addRot(dx, dy);
        this.currentMethod.updateReferences();
    }

    addMove(dx, dy) {
        this.currentMethod.addMove(dx, dy);
        this.currentMethod.updateReferences();
    }

    addZoom(dz) {
        this.currentMethod.addZoom(dz);
        this.currentMethod.updateReferences();
    }

    updateSmoothness(fps) {
        let tau = 4.5 / fps;  // 4.5 <- fps:30[/s], tau:15s is best
        this.tau = 0.9 * this.tau + 0.1 * tau;
    }

    update(delta) {
        let factor = (1 - Math.exp(-delta / this.tau));

        this.currentMethod.updateReferences();
        this.currentMethod.computeReferences();
        let refRot = this.currentMethod.getReferenceRot();
        let refVp = this.currentMethod.getReferenceVp();
        let refDist = this.currentMethod.getReferenceDist();

        // Compute residuals
        let resPh = refRot.x - this.currentRot.x;
        let resTh = refRot.y - this.currentRot.y;
        let resPs = refRot.z - this.currentRot.z;

        if (resTh > Math.PI) {
            resTh -= Math.PI * 2;
            this.currentRot.y += Math.PI * 2
        }
        if (resTh < -Math.PI) {
            resTh += Math.PI * 2;
            this.currentRot.y -= Math.PI * 2
        }

        let resX = refVp.x - this.currentVp.x;
        let resY = refVp.y - this.currentVp.y;
        let resZ = refVp.z - this.currentVp.z;

        let resDist = refDist - this.currentDist;

        // Update variables
        this.currentRot.x += resPh * factor;
        this.currentRot.y += resTh * factor;
        this.currentRot.z += resPs * factor;
        this.currentVp.x += resX * factor;
        this.currentVp.y += resY * factor;
        this.currentVp.z += resZ * factor;
        this.currentDist += resDist * factor;

        // Apply to camera.
        this.camera.rotation.x = this.currentRot.x;
        this.camera.rotation.y = this.currentRot.y;
        this.camera.rotation.z = this.currentRot.z;

        let eye = new THREE.Vector3(0, 0, 0);
        eye.x = -Math.cos(this.currentRot.x) * Math.sin(this.currentRot.y);
        eye.y = Math.sin(this.currentRot.x);
        eye.z = -Math.cos(this.currentRot.x) * Math.cos(this.currentRot.y);

        let center = new THREE.Vector3();
        center.copy(this.currentVp);
        center.x -= eye.x * this.currentDist;
        center.y -= eye.y * this.currentDist;
        center.z -= eye.z * this.currentDist;
        this.camera.position.x = center.x;
        this.camera.position.y = center.y;
        this.camera.position.z = center.z;
    }
}
