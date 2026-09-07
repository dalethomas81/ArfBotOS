"""Benchmark harness for FastTemplateMatching. Not part of the vision program."""
import os
import sys
import time
import cProfile
import pstats
import io
import traceback

import cv2
import numpy

import FastTemplateMatching as ftm


def identity_cal():
    origin = numpy.array([[0.0], [0.0]], dtype=numpy.float32)
    return 1.0, 0.0, origin


def load_cal_roi(folder):
    cal_path = os.path.join(folder, "cal.yaml")
    roi_path = os.path.join(folder, "roi.yaml")
    if os.path.isfile(cal_path):
        _, _, _, _, pixel_ratio, rotation_offset, _, _, origin = ftm.load_cal_data(cal_path)
    else:
        pixel_ratio, rotation_offset, origin = identity_cal()
    if os.path.isfile(roi_path):
        roi_tl, roi_br = ftm.load_roi_data(roi_path)
    else:
        roi_tl = roi_br = None
    return pixel_ratio, rotation_offset, origin, roi_tl, roi_br


def find_pair(folder, names):
    for name in names:
        path = os.path.join(folder, name)
        if os.path.isfile(path):
            return path
    return None


def run_case(name, src_path, templ_path, folder, iMaxPos=2, dMaxOverlap=0.0, dScore=0.6, dToleranceAngle=90.0, save=None):
    src = cv2.imread(src_path, cv2.IMREAD_GRAYSCALE)
    templ = cv2.imread(templ_path, cv2.IMREAD_GRAYSCALE)
    if src is None or templ is None:
        print(f"SKIP {name}: failed to load images")
        return None
    pixel_ratio, rotation_offset, origin, roi_tl, roi_br = load_cal_roi(folder)
    if roi_tl is None:
        roi_tl = numpy.array([[0.0], [0.0]], dtype=numpy.float32)
        roi_br = numpy.array([[float(src.shape[1])], [float(src.shape[0])]], dtype=numpy.float32)
    if save is None:
        out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_bench_out")
        os.makedirs(out_dir, exist_ok=True)
        save = os.path.join(out_dir, name.replace("/", "_") + ".bmp")
    ftm.m_vecSingleTargetData = []
    t0 = time.perf_counter()
    try:
        ftm.main(
            src, templ, save, iMaxPos, dMaxOverlap, dScore, dToleranceAngle,
            pixel_ratio, rotation_offset, origin, roi_tl, roi_br, True,
        )
        ok = True
        err = None
    except Exception as e:
        ok = False
        err = traceback.format_exc()
    dt = time.perf_counter() - t0
    n = len(ftm.m_vecSingleTargetData)
    print(f"CASE {name}: {dt:.3f}s  matches={n}  src={src.shape[1]}x{src.shape[0]}  tmpl={templ.shape[1]}x{templ.shape[0]}  ok={ok}")
    if err:
        print(err)
    return dt, n, ok


def main():
    base = os.path.dirname(os.path.abspath(__file__))
    ti = os.path.join(base, "Test Images")
    cases = [
        ("Test5", os.path.join(base, "Test5"), ["Input.jpg"], ["Template.jpg"], dict(iMaxPos=1, dScore=0.6, dToleranceAngle=30.0)),
        ("Test6", os.path.join(base, "Test6"), ["Input.jpg"], ["battery.jpg"], dict(iMaxPos=1, dScore=0.6, dToleranceAngle=90.0)),
        ("Test3", os.path.join(base, "Test3"), ["Input.bmp"], ["Template.bmp"], dict(iMaxPos=2, dScore=0.6, dToleranceAngle=90.0)),
        ("Test2", os.path.join(base, "Test2"), ["Input.bmp"], ["Template.bmp"], dict(iMaxPos=2, dScore=0.5, dToleranceAngle=90.0)),
        ("Test1", os.path.join(base, "Test1"), ["Input.bmp"], ["Template.bmp"], dict(iMaxPos=5, dScore=0.6, dToleranceAngle=90.0)),
        ("Test4", os.path.join(base, "Test4"), ["Input.jpg"], ["Template.jpg"], dict(iMaxPos=2, dScore=0.8, dToleranceAngle=90.0)),
        ("Src8-Dst8", ti, ["Src8.bmp"], ["Dst8.bmp"], dict(iMaxPos=2, dScore=0.6, dToleranceAngle=90.0)),
        ("Src9-Dst9", ti, ["Src9.bmp"], ["Dst9.bmp"], dict(iMaxPos=2, dScore=0.6, dToleranceAngle=90.0)),
        ("Src4-Dst4", ti, ["Src4.bmp"], ["Dst4.bmp"], dict(iMaxPos=1, dScore=0.5, dToleranceAngle=90.0)),
        ("Src1-Dst2", ti, ["Src1.bmp"], ["Dst2.bmp"], dict(iMaxPos=5, dScore=0.6, dToleranceAngle=90.0)),
        ("Src5-0", ti, ["Src5-0.bmp"], ["Dst5.bmp"], dict(iMaxPos=1, dScore=0.6, dToleranceAngle=180.0)),
        ("Src5-45", ti, ["Src5-45.bmp"], ["Dst5.bmp"], dict(iMaxPos=1, dScore=0.6, dToleranceAngle=180.0)),
        ("Src5-90", ti, ["Src5-90.bmp"], ["Dst5.bmp"], dict(iMaxPos=1, dScore=0.6, dToleranceAngle=180.0)),
        ("Src5-180", ti, ["Src5-180.bmp"], ["Dst5.bmp"], dict(iMaxPos=1, dScore=0.6, dToleranceAngle=180.0)),
        ("M12-D", os.path.join(ti, "M12"), ["M12_D_Test.jpg"], ["D.jpg"], dict(iMaxPos=5, dScore=0.6, dToleranceAngle=20.0)),
        ("Src6-Dst6", ti, ["Src6.jpg"], ["Dst6.bmp"], dict(iMaxPos=1, dScore=0.6, dToleranceAngle=45.0)),
    ]
    profile_name = os.environ.get("BENCH_PROFILE")
    if profile_name:
        cases = [c for c in cases if c[0] == profile_name]
        pr = cProfile.Profile()
        pr.enable()
    for name, folder, srcs, tmpls, kwargs in cases:
        src = find_pair(folder, srcs)
        tmpl = find_pair(folder, tmpls)
        if not src or not tmpl:
            print(f"SKIP {name}: missing files")
            continue
        run_case(name, src, tmpl, folder, **kwargs)
    if profile_name:
        pr.disable()
        s = io.StringIO()
        pstats.Stats(pr, stream=s).sort_stats("cumulative").print_stats(40)
        print(s.getvalue())


if __name__ == "__main__":
    main()
