/* ArfBotOS RobotAnimator — CODESYS WebVisu HTML5 control.
 * Generic industrial 6-axis body (cylinders / housings / wrist / flange).
 * Joint inputs are degrees (SoftMotion axis fActPosition).
 * DH matches SMC_TrafoConfig_ArticulatedRobot_6DOF (millimetres).
 */
var RobotAnimatorElementWrapper;

(function () {
    "use strict";

    var DEG = Math.PI / 180;
    var CANVAS = "#0E141B";
    var ACCENT = "#2EE6D6";
    var SUCCESS = "#3DDC84";
    var WARNING = "#FFB020";
    var MUTED = "#8B9BB0";
    var GRID = "#243040";
    var TEXT = "#E8EEF5";
    var METAL = { r: 168, g: 178, b: 188 };
    var HOUSING = { r: 52, g: 62, b: 74 };
    var DARK = { r: 28, g: 34, b: 42 };
    var RING = { r: 46, g: 230, b: 214 };

    var DH = {
        a1: 64.2,
        a2: 305.0,
        a3: 0.0,
        d1: -169.77,
        d3: 0.0,
        d4: 222.63,
        d6: 36.25
    };

    // Map SoftMotion fActPosition (deg) to DH theta. J2 +90 is the 6-DOF convention.
    // J4 sign matches the physical wrist; J6 +90 aligns the gripper fingers with the flange.
    var AXIS_SIGN = [1, 1, 1, -1, 1, 1];
    var AXIS_OFFSET = [0, 90, 0, 0, 0, 90];

    var LIGHT = { x: 0.35, y: 0.55, z: 0.75 };

    function mul4(a, b) {
        var r = new Array(16);
        var i, j, k;
        for (i = 0; i < 4; i++) {
            for (j = 0; j < 4; j++) {
                r[i * 4 + j] = 0;
                for (k = 0; k < 4; k++) {
                    r[i * 4 + j] += a[i * 4 + k] * b[k * 4 + j];
                }
            }
        }
        return r;
    }

    function dh(thetaDeg, d, a, alphaDeg) {
        var th = thetaDeg * DEG;
        var al = alphaDeg * DEG;
        var ct = Math.cos(th);
        var st = Math.sin(th);
        var ca = Math.cos(al);
        var sa = Math.sin(al);
        return [
            ct, -st * ca,  st * sa, a * ct,
            st,  ct * ca, -ct * sa, a * st,
            0,   sa,       ca,      d,
            0,   0,        0,       1
        ];
    }

    function xform(m, x, y, z) {
        return {
            x: m[0] * x + m[1] * y + m[2] * z + m[3],
            y: m[4] * x + m[5] * y + m[6] * z + m[7],
            z: m[8] * x + m[9] * y + m[10] * z + m[11]
        };
    }

    function origin(m) {
        return { x: m[3], y: m[7], z: m[11] };
    }

    function axisX(m) { return { x: m[0], y: m[4], z: m[8] }; }
    function axisY(m) { return { x: m[1], y: m[5], z: m[9] }; }
    function axisZ(m) { return { x: m[2], y: m[6], z: m[10] }; }

    function add(a, b) { return { x: a.x + b.x, y: a.y + b.y, z: a.z + b.z }; }
    function sub(a, b) { return { x: a.x - b.x, y: a.y - b.y, z: a.z - b.z }; }
    function scale(a, s) { return { x: a.x * s, y: a.y * s, z: a.z * s }; }
    function dot(a, b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
    function cross(a, b) {
        return {
            x: a.y * b.z - a.z * b.y,
            y: a.z * b.x - a.x * b.z,
            z: a.x * b.y - a.y * b.x
        };
    }
    function len(a) { return Math.sqrt(dot(a, a)); }
    function norm(a) {
        var l = len(a);
        if (l < 1e-9) {
            return { x: 0, y: 0, z: 1 };
        }
        return scale(a, 1 / l);
    }
    function lerp(a, b, t) { return add(a, scale(sub(b, a), t)); }

    function worldUp() {
        return [
            1, 0, 0, 0,
            0, -1, 0, 0,
            0, 0, -1, 0,
            0, 0, 0, 1
        ];
    }

    function frames(q) {
        var T = worldUp();
        var list = [T];
        var th0 = AXIS_SIGN[0] * q[0] + AXIS_OFFSET[0];
        var th1 = AXIS_SIGN[1] * q[1] + AXIS_OFFSET[1];
        var th2 = AXIS_SIGN[2] * q[2] + AXIS_OFFSET[2];
        var th3 = AXIS_SIGN[3] * q[3] + AXIS_OFFSET[3];
        var th4 = AXIS_SIGN[4] * q[4] + AXIS_OFFSET[4];
        var th5 = AXIS_SIGN[5] * q[5] + AXIS_OFFSET[5];
        var steps = [
            [th0, DH.d1, DH.a1, -90],
            [th1, 0, DH.a2, 0],
            [th2, DH.d3, DH.a3, 90],
            [th3, DH.d4, 0, 90],
            [th4, 0, 0, -90],
            [th5, DH.d6, 0, 0]
        ];
        var i;
        for (i = 0; i < 6; i++) {
            T = mul4(T, dh(steps[i][0], steps[i][1], steps[i][2], steps[i][3]));
            list.push(T);
        }
        return list;
    }

    function rgb(c, shade) {
        var s = shade;
        if (s < 0.18) { s = 0.18; }
        if (s > 1.15) { s = 1.15; }
        var r = Math.round(c.r * s);
        var g = Math.round(c.g * s);
        var b = Math.round(c.b * s);
        if (r > 255) { r = 255; }
        if (g > 255) { g = 255; }
        if (b > 255) { b = 255; }
        return "rgb(" + r + "," + g + "," + b + ")";
    }

    function shadeAlong(dir) {
        var n = norm(dir);
        return 0.45 + 0.55 * Math.max(0, dot(n, LIGHT));
    }

    RobotAnimatorElementWrapper = function (idGenerator, maybeHeight) {
        var self = this;
        this.joints = [0, 0, 0, 0, 0, 0];
        this.gripper = 1;
        this.yaw = 0.95 + Math.PI;
        this.pitch = 0.55;
        this.distance = 1400;
        this._zoom = 1;
        this._lookAt = { x: 40, y: 0, z: 120 };
        this._error = "";
        this.dragging = false;
        this.lastX = 0;
        this.lastY = 0;
        this._raf = 0;
        this._cssW = 1;
        this._cssH = 1;
        this._hudHtml = "";
        this._initW = 0;
        this._initH = 0;
        if (typeof idGenerator === "number") {
            this._initW = idGenerator;
            this._initH = typeof maybeHeight === "number" ? maybeHeight : 0;
        }

        if (document.documentElement) {
            document.documentElement.style.cssText = "width:100%;height:100%;margin:0;padding:0;";
        }
        if (document.body) {
            document.body.style.cssText = "width:100%;height:100%;margin:0;padding:0;overflow:hidden;background:" + CANVAS + ";";
        }

        this.domNode = document.createElement("div");
        this.domNode.style.cssText = "position:absolute;left:0;top:0;right:0;bottom:0;width:100%;height:100%;overflow:hidden;background:" + CANVAS + ";font-family:'Segoe UI',sans-serif;";
        this.parentNode = this.domNode;

        this.canvas = document.createElement("canvas");
        this.canvas.style.cssText = "display:block;position:absolute;left:0;top:0;width:100%;height:100%;touch-action:none;";
        this.domNode.appendChild(this.canvas);
        this.ctx = this.canvas.getContext("2d");

        this.hud = document.createElement("div");
        this.hud.style.cssText = "position:absolute;left:8px;top:8px;z-index:2;color:" + TEXT + ";font-size:11px;line-height:1.45;pointer-events:none;text-shadow:0 1px 2px #000;";
        this.domNode.appendChild(this.hud);

        document.body.appendChild(this.domNode);

        this.canvas.addEventListener("pointerdown", function (e) {
            self.dragging = true;
            self.lastX = e.clientX;
            self.lastY = e.clientY;
            try { self.canvas.setPointerCapture(e.pointerId); } catch (err) {}
        });
        this.canvas.addEventListener("pointermove", function (e) {
            if (!self.dragging) {
                return;
            }
            var dx = e.clientX - self.lastX;
            var dy = e.clientY - self.lastY;
            self.lastX = e.clientX;
            self.lastY = e.clientY;
            self.yaw -= dx * 0.008;
            self.pitch += dy * 0.008;
            if (self.pitch > 1.45) { self.pitch = 1.45; }
            if (self.pitch < 0.08) { self.pitch = 0.08; }
        });
        this.canvas.addEventListener("pointerup", function () { self.dragging = false; });
        this.canvas.addEventListener("pointercancel", function () { self.dragging = false; });
        this.canvas.addEventListener("wheel", function (e) {
            e.preventDefault();
            self._zoom *= (e.deltaY > 0) ? 1.08 : 0.92;
            if (self._zoom < 0.45) { self._zoom = 0.45; }
            if (self._zoom > 2.8) { self._zoom = 2.8; }
        }, { passive: false });
        this.canvas.addEventListener("dblclick", function () {
            self._zoom = 1;
        });

        window.addEventListener("resize", function () { self._draw(); });

        this._loop = function () {
            self._draw();
            self._raf = window.requestAnimationFrame(self._loop);
        };
        this._loop();
    };

    RobotAnimatorElementWrapper.prototype._hostSize = function () {
        var w = window.innerWidth || 0;
        var h = window.innerHeight || 0;
        if (w < 2) { w = this._initW || 0; }
        if (h < 2) { h = this._initH || 0; }
        if (w < 2 && this.domNode) { w = this.domNode.clientWidth || 0; }
        if (h < 2 && this.domNode) { h = this.domNode.clientHeight || 0; }
        if (w < 2 && document.documentElement) { w = document.documentElement.clientWidth || 0; }
        if (h < 2 && document.documentElement) { h = document.documentElement.clientHeight || 0; }
        if (w < 2) { w = 320; }
        if (h < 2) { h = 240; }
        return { w: w, h: h };
    };

    RobotAnimatorElementWrapper.prototype._fitCamera = function (parts) {
        var look = { x: 40, y: 0, z: 120 };
        var i, dx, dy, dz, d, maxR, shortSide;
        maxR = 280;
        function acc(q) {
            if (!q) { return; }
            dx = q.x - look.x;
            dy = q.y - look.y;
            dz = q.z - look.z;
            d = Math.sqrt(dx * dx + dy * dy + dz * dz);
            if (d > maxR) { maxR = d; }
        }
        acc({ x: 0, y: 0, z: 0 });
        acc({ x: 140, y: 140, z: 0 });
        acc({ x: -140, y: -140, z: 0 });
        for (i = 0; i < parts.length; i++) {
            if (parts[i].kind === "ball") {
                acc(parts[i].p);
            } else {
                acc(parts[i].a);
                acc(parts[i].b);
            }
        }
        this._lookAt = look;
        shortSide = Math.min(this._cssW, this._cssH);
        if (shortSide < 80) { shortSide = 80; }
        this.distance = maxR * 850 / (shortSide * 0.38) * this._zoom;
        if (this.distance < 1200) { this.distance = 1200; }
        if (this.distance > 8000) { this.distance = 8000; }
    };

    RobotAnimatorElementWrapper.prototype._setJoint = function (i, value) {
        var n = Number(value);
        if (!isFinite(n)) {
            return;
        }
        this.joints[i] = n;
    };

    RobotAnimatorElementWrapper.prototype.setJ1 = function (value) { this._setJoint(0, value); };
    RobotAnimatorElementWrapper.prototype.setJ2 = function (value) { this._setJoint(1, value); };
    RobotAnimatorElementWrapper.prototype.setJ3 = function (value) { this._setJoint(2, value); };
    RobotAnimatorElementWrapper.prototype.setJ4 = function (value) { this._setJoint(3, value); };
    RobotAnimatorElementWrapper.prototype.setJ5 = function (value) { this._setJoint(4, value); };
    RobotAnimatorElementWrapper.prototype.setJ6 = function (value) { this._setJoint(5, value); };

    RobotAnimatorElementWrapper.prototype.setGripper = function (value) {
        var n = Number(value);
        if (!isFinite(n)) {
            return;
        }
        if (n < 0) { n = 0; }
        if (n <= 1) {
            this.gripper = n;
            return;
        }
        if (n > 100) { n = 100; }
        this.gripper = n / 100;
    };

    RobotAnimatorElementWrapper.prototype._project = function (p) {
        var look = this._lookAt || { x: 0, y: 0, z: 0 };
        var px = p.x - look.x;
        var py = p.y - look.y;
        var pz = p.z - look.z;
        var cy = Math.cos(this.yaw);
        var sy = Math.sin(this.yaw);
        var cp = Math.cos(this.pitch);
        var sp = Math.sin(this.pitch);
        var x = px * cy + py * sy;
        var z = -px * sy + py * cy;
        var y = pz;
        var y2 = y * cp - z * sp;
        var z2 = y * sp + z * cp;
        var camZ = z2 - this.distance;
        var s = 850 / Math.max(40, -camZ);
        return {
            x: this._cssW * 0.5 + x * s,
            y: this._cssH * 0.58 - y2 * s,
            depth: camZ,
            s: s
        };
    };

    RobotAnimatorElementWrapper.prototype._cameraPos = function () {
        var look = this._lookAt || { x: 0, y: 0, z: 0 };
        var cy = Math.cos(this.yaw);
        var sy = Math.sin(this.yaw);
        var cp = Math.cos(this.pitch);
        var sp = Math.sin(this.pitch);
        var d = this.distance;
        return {
            x: look.x - sy * cp * d,
            y: look.y + cy * cp * d,
            z: look.z + sp * d
        };
    };

    RobotAnimatorElementWrapper.prototype._perpBasis = function (axis) {
        var n = norm(axis);
        var u, v, a, b;
        if (n.z < -0.999999) {
            u = { x: 0, y: -1, z: 0 };
            v = { x: -1, y: 0, z: 0 };
        } else {
            a = 1 / (1 + n.z);
            b = -n.x * n.y * a;
            u = { x: 1 - n.x * n.x * a, y: b, z: -n.x };
            v = { x: b, y: 1 - n.y * n.y * a, z: -n.y };
        }
        return { u: u, v: v, n: n };
    };

    RobotAnimatorElementWrapper.prototype._ring = function (center, axis, radius, n) {
        var b = this._perpBasis(axis);
        var pts = [];
        var i, ang;
        for (i = 0; i < n; i++) {
            ang = (i / n) * Math.PI * 2;
            pts.push(add(center, add(scale(b.u, Math.cos(ang) * radius), scale(b.v, Math.sin(ang) * radius))));
        }
        return pts;
    };

    RobotAnimatorElementWrapper.prototype._pushCyl = function (parts, a, b, radius, color) {
        var d = sub(b, a);
        var L = len(d);
        var nSeg, i, t0, t1;
        if (L < 0.5) {
            parts.push({ kind: "ball", p: a, r: radius, color: color, id: parts.length });
            return;
        }
        nSeg = Math.ceil(L / 36);
        if (nSeg < 1) { nSeg = 1; }
        if (nSeg > 10) { nSeg = 10; }
        for (i = 0; i < nSeg; i++) {
            t0 = i / nSeg;
            t1 = (i + 1) / nSeg;
            parts.push({
                kind: "cyl",
                a: lerp(a, b, t0),
                b: lerp(a, b, t1),
                r: radius,
                color: color,
                capA: i === 0,
                capB: i === nSeg - 1,
                id: parts.length
            });
        }
    };

    RobotAnimatorElementWrapper.prototype._pushBall = function (parts, p, radius, color) {
        parts.push({
            kind: "ball",
            p: p,
            r: radius,
            color: color,
            id: parts.length
        });
    };

    RobotAnimatorElementWrapper.prototype._drawCyl = function (part) {
        var ctx = this.ctx;
        var n = 20;
        var ringA = this._ring(part.a, sub(part.b, part.a), part.r, n);
        var ringB = this._ring(part.b, sub(part.b, part.a), part.r, n);
        var i, pa, pb, pc, pd, mid, lit, axis;
        axis = norm(sub(part.b, part.a));
        for (i = 0; i < n; i++) {
            pa = this._project(ringA[i]);
            pb = this._project(ringA[(i + 1) % n]);
            pc = this._project(ringB[(i + 1) % n]);
            pd = this._project(ringB[i]);
            if ((pb.x - pa.x) * (pd.y - pa.y) - (pb.y - pa.y) * (pd.x - pa.x) <= 0) {
                continue;
            }
            mid = lerp(ringA[i], ringB[i], 0.5);
            lit = shadeAlong(sub(mid, lerp(part.a, part.b, 0.5)));
            ctx.beginPath();
            ctx.moveTo(pa.x, pa.y);
            ctx.lineTo(pb.x, pb.y);
            ctx.lineTo(pc.x, pc.y);
            ctx.lineTo(pd.x, pd.y);
            ctx.closePath();
            ctx.fillStyle = rgb(part.color, lit);
            ctx.fill();
        }
        if (part.capA !== false) {
            this._drawCap(part.a, axis, part.r, part.color, -1);
        }
        if (part.capB !== false) {
            this._drawCap(part.b, axis, part.r, part.color, 1);
        }
    };

    RobotAnimatorElementWrapper.prototype._drawCap = function (center, axis, radius, color, sign) {
        var nrm = scale(axis, sign);
        var cam = this._cameraPos();
        if (dot(nrm, sub(cam, center)) <= 0.5) {
            return;
        }
        var ctx = this.ctx;
        var ring = this._ring(center, axis, radius, 20);
        var i, p0, p;
        p0 = this._project(ring[0]);
        ctx.beginPath();
        ctx.moveTo(p0.x, p0.y);
        for (i = 1; i < ring.length; i++) {
            p = this._project(ring[i]);
            ctx.lineTo(p.x, p.y);
        }
        ctx.closePath();
        ctx.fillStyle = rgb(color, shadeAlong(scale(axis, sign)) * 0.95);
        ctx.fill();
    };

    RobotAnimatorElementWrapper.prototype._drawBall = function (part) {
        var pr = this._project(part.p);
        var ctx = this.ctx;
        var r = Math.max(1.5, part.r * pr.s);
        if (!isFinite(pr.x) || !isFinite(pr.y) || !isFinite(r)) {
            return;
        }
        ctx.beginPath();
        ctx.arc(pr.x, pr.y, r, 0, Math.PI * 2);
        ctx.fillStyle = rgb(part.color, 0.85);
        ctx.fill();
        ctx.strokeStyle = rgb(part.color, 0.45);
        ctx.lineWidth = 1;
        ctx.stroke();
    };

    RobotAnimatorElementWrapper.prototype._grid = function () {
        var ctx = this.ctx;
        var i, a, b, pa, pb;
        var step = 50;
        var n = 8;
        ctx.lineWidth = 1;
        ctx.strokeStyle = GRID;
        for (i = -n; i <= n; i++) {
            a = this._project({ x: i * step, y: -n * step, z: 0 });
            b = this._project({ x: i * step, y: n * step, z: 0 });
            ctx.beginPath();
            ctx.moveTo(a.x, a.y);
            ctx.lineTo(b.x, b.y);
            ctx.stroke();
            a = this._project({ x: -n * step, y: i * step, z: 0 });
            b = this._project({ x: n * step, y: i * step, z: 0 });
            ctx.beginPath();
            ctx.moveTo(a.x, a.y);
            ctx.lineTo(b.x, b.y);
            ctx.stroke();
        }
        function axisLine(from, to, color) {
            pa = this._project(from);
            pb = this._project(to);
            ctx.beginPath();
            ctx.moveTo(pa.x, pa.y);
            ctx.lineTo(pb.x, pb.y);
            ctx.strokeStyle = color;
            ctx.lineWidth = 2;
            ctx.stroke();
        }
        axisLine.call(this, { x: 0, y: 0, z: 0 }, { x: 90, y: 0, z: 0 }, "#FF4D6A");
        axisLine.call(this, { x: 0, y: 0, z: 0 }, { x: 0, y: 90, z: 0 }, SUCCESS);
        axisLine.call(this, { x: 0, y: 0, z: 0 }, { x: 0, y: 0, z: 90 }, ACCENT);
    };

    RobotAnimatorElementWrapper.prototype._housing = function (parts, frame, along, halfH, radius, color) {
        var o = origin(frame);
        var ax = along(frame);
        this._pushCyl(parts, add(o, scale(ax, -halfH)), add(o, scale(ax, halfH)), radius, color);
    };

    RobotAnimatorElementWrapper.prototype._bodyParts = function (fs) {
        var parts = [];
        var o0 = origin(fs[0]);
        var o1 = origin(fs[1]);
        var o2 = origin(fs[2]);
        var o3 = origin(fs[3]);
        var o4 = origin(fs[4]);
        var o5 = origin(fs[5]);
        var o6 = origin(fs[6]);
        var up = { x: 0, y: 0, z: 1 };
        var pedestalTop = { x: o0.x, y: o0.y, z: o0.z + Math.abs(DH.d1) };

        this._pushCyl(parts, add(o0, scale(up, -6)), add(o0, scale(up, 12)), 82, DARK);
        this._pushCyl(parts, add(o0, scale(up, 10)), pedestalTop, 38, HOUSING);
        this._pushCyl(parts, add(pedestalTop, scale(up, -12)), add(pedestalTop, scale(up, 24)), 46, HOUSING);
        this._pushCyl(parts, add(pedestalTop, scale(up, 18)), add(pedestalTop, scale(up, 26)), 48, RING);

        this._housing(parts, fs[1], axisZ, 22, 36, HOUSING);
        this._pushCyl(parts, o1, o2, 24, METAL);
        this._housing(parts, fs[1], axisY, 32, 30, HOUSING);

        this._housing(parts, fs[2], axisZ, 20, 30, HOUSING);
        this._pushCyl(parts, o2, o3, 20, METAL);

        this._housing(parts, fs[3], axisZ, 18, 26, HOUSING);
        this._pushCyl(parts, o3, o4, 16, METAL);

        this._housing(parts, fs[4], axisZ, 16, 20, HOUSING);
        this._pushBall(parts, o4, 22, HOUSING);
        this._pushCyl(parts, o4, o5, 14, METAL);

        this._housing(parts, fs[5], axisZ, 14, 16, HOUSING);
        this._pushCyl(parts, o5, o6, 12, METAL);

        this._housing(parts, fs[6], axisZ, 10, 18, DARK);
        this._addGripper(parts, fs[6]);

        return parts;
    };

    RobotAnimatorElementWrapper.prototype._addGripper = function (parts, frame) {
        var o = origin(frame);
        var x = axisX(frame);
        var z = axisZ(frame);
        var g = this.gripper;
        if (g < 0) { g = 0; }
        if (g > 1) { g = 1; }
        var closed = 7;
        var opened = 26;
        var spread = closed + (opened - closed) * g;
        var palm0 = add(o, scale(z, 8));
        var palm1 = add(o, scale(z, 28));
        this._pushCyl(parts, palm0, palm1, 16, DARK);
        this._pushCyl(parts, add(palm1, scale(x, -spread)), add(palm1, scale(x, spread)), 5, HOUSING);
        var s, knuckle, finger0, finger1, pad0, pad1;
        for (s = -1; s <= 1; s += 2) {
            knuckle = add(palm1, scale(x, s * spread));
            finger0 = add(knuckle, scale(z, 2));
            finger1 = add(knuckle, scale(z, 42));
            this._pushCyl(parts, finger0, finger1, 5, METAL);
            pad0 = add(add(finger1, scale(z, -10)), scale(x, -s * 4));
            pad1 = add(add(finger1, scale(z, 4)), scale(x, -s * 4));
            this._pushCyl(parts, pad0, pad1, 4, RING);
            this._pushBall(parts, finger1, 5, HOUSING);
        }
    };

    RobotAnimatorElementWrapper.prototype._draw = function () {
        var canvas = this.canvas;
        var dpr = window.devicePixelRatio || 1;
        var size = this._hostSize();
        var w = Math.max(1, size.w);
        var h = Math.max(1, size.h);
        this._cssW = w;
        this._cssH = h;
        if (canvas.width !== Math.floor(w * dpr) || canvas.height !== Math.floor(h * dpr)) {
            canvas.width = Math.floor(w * dpr);
            canvas.height = Math.floor(h * dpr);
            canvas.style.width = w + "px";
            canvas.style.height = h + "px";
        }
        var ctx = this.ctx;
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        ctx.fillStyle = CANVAS;
        ctx.fillRect(0, 0, w, h);

        var fs, parts, i, lines, html;
        try {
            fs = frames(this.joints);
            parts = this._bodyParts(fs);
            this._fitCamera(parts);
            this._grid();
            for (i = 0; i < parts.length; i++) {
                if (parts[i].kind === "ball") {
                    parts[i].depth = this._project(parts[i].p).depth;
                } else {
                    parts[i].depth = (this._project(parts[i].a).depth + this._project(parts[i].b).depth) * 0.5;
                }
            }
            parts.sort(function (a, b) {
                var d = a.depth - b.depth;
                if (d < -0.8) { return -1; }
                if (d > 0.8) { return 1; }
                return a.id - b.id;
            });
            for (i = 0; i < parts.length; i++) {
                if (parts[i].kind === "cyl") {
                    this._drawCyl(parts[i]);
                } else {
                    this._drawBall(parts[i]);
                }
            }
            this._error = "";
        } catch (err) {
            this._error = String(err && err.message ? err.message : err);
        }

        lines = ["Generic 6-axis"];
        for (i = 0; i < 6; i++) {
            lines.push("J" + (i + 1) + "  " + this.joints[i].toFixed(1) + "\u00B0");
        }
        lines.push("Grip  " + Math.round(this.gripper * 100) + "% open");
        if (this._error) {
            lines.push(this._error);
        }
        html = lines.join("<br>");
        if (html !== this._hudHtml) {
            this._hudHtml = html;
            this.hud.innerHTML = html;
        }
    };
})();
