# Verification record — 2026-10-17

- Gradle `build` succeeded using WPILib 2026 JDK 17. All 13 Java tests passed, including command-level feed cancellation. The initial command test needed an extra execute cycle to advance the beforeStarting sequential wrapper; no readiness timeout was relaxed.
- The built `code.jar` contains `deployment-manifest.json` (15,468 bytes at verification).
- Bundled Python suite: 41 tests passed. Video HTTP tests use a temporary fixture, so broadcasts are not required for tests. Log-analysis reproduction still requires the original logs.
- Root research `scripts/validate.py`: pass, zero errors, four existing cross-source result conflicts (2022/2767, 2024/1261, 2026/1690, 2026/2231). No KRAIL topics were rebuilt in this implementation task.
- Detector: 13 competition logs decoded without reported decode errors; 572 launch candidates, 87 intake contacts. Only six launch candidates meet the exploratory stable-voltage filter; this does not establish six real shots.
- Video evidence: two Q48 windows sampled at 1 fps with full-resolution central frames; both unresolved for individual 1257 crossings. Source URL, video hash, sampled times and observations are retained.
- Browser: loaded local video, filtered Q48/intake, navigated to log 168.554 s, verified seek to video 31.368 s (two seconds before candidate), saved an uncertain label and verified it survived event navigation. Export control invoked; observations are separately retained as JSON.
- No physical robot deployment, full-match video count, end-to-end autonomous physics validation, CAN fault injection on hardware, or measured scoring improvement was performed.
