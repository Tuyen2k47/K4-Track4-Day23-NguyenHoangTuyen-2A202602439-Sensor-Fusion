# Báo cáo bài nộp — Day 23 Sensor Fusion Lab

> Điền file này rồi commit. Cách nộp: [hướng dẫn nộp](../SUBMISSION.md).

## Thông tin học viên

- Họ tên: Nguyễn Hoàng Tuyên
- MSSV: 2A202602439
- Email: 26ai.tuyennh3@vinuni.edu.vn
- Link repo (fork): https://github.com/Tuyen2k47/K4-Track4-Day23-NguyenHoangTuyen-2A202602439-Sensor-Fusion
- Commit hash nộp (`git rev-parse HEAD`): 4771711d6ca73a754b64006a9710ef5966e93bcf


## Tóm tắt kết quả

- `fusion_mode` (bắt buộc `compare`), `frames`, `segment`, `seed`: fusion_mode: compare, frames: [0, 198], segment: training_segment-1005081002024129653_5313_150_5333_150_with_camera_labels.tfrecord, seed: 0
- `detection.precision`, `detection.recall`, `detection.tp/fp/fn`: precision: 0.9701, recall: 0.7004, tp: 519, fp: 16, fn: 222
- `tracking.lidar.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: rmse: 0.1503, matches: 502, sum_sq_err: 11.3369, ghost_track_frames: 0, missed_gt_frames: 239, mean_confirmed_tracks: 2.5226
- `tracking.fused.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: rmse: 0.1358, matches: 502, sum_sq_err: 9.2642, ghost_track_frames: 0, missed_gt_frames: 239, mean_confirmed_tracks: 2.5226
- Giải thích khác biệt hai mode, đọc RMSE cùng số ghép và ghost/miss:
  Ở cả hai chế độ (lidar-only và fused), số cặp ghép thành công (`matches` = 502), số ghost tracks (`ghost_track_frames` = 0), số missed ground truth (`missed_gt_frames` = 239) và số confirmed tracks trung bình (`mean_confirmed_tracks` = 2.5226) đều giống hệt nhau. Điều này hoàn toàn phù hợp với thiết kế track-then-fuse: camera pass chỉ cập nhật trạng thái EKF (tinh chỉnh vị trí và vận tốc) chứ không tham gia vào vòng đời track (không tạo mới, không xóa, không thay đổi hit/miss score). Về độ chính xác, chế độ `fused` đạt RMSE = 0.1358 m, giảm đáng kể (tốt hơn ~0.0145 m, tương đương giảm ~10% sai số vị trí 3D so với LiDAR-only đạt RMSE = 0.1503 m). Cả hai chế độ đều vượt xa ngưỡng yêu cầu chuẩn (RMSE <= 0.45 m, độ lệch rmse_fused - rmse_lidar <= 0.05 m). Việc đọc RMSE cùng với matches và ghost/miss khẳng định hệ thống tracking duy trì liên tục tính ổn định và nhất quán, không hề sinh ra vệt ảo (ghost = 0) trong khi bổ sung thông tin từ camera giúp hội tụ ước lượng vị trí chính xác hơn.

Chạy từ root repo:

```bash
fusion-run-lab --config student/config/paths.yaml --fusion compare --seed 0
```

`rmse = sqrt(sum_sq_err/matches)` trên vị trí 3D của confirmed tracks ghép
một-một với GT xe trong cửa sổ BEV, gate XY **2.0 m**; `null` nếu không có cặp.
Camera dùng tâm hộp 2D ground-truth FRONT có nhiễu seeded, **không** dùng camera
detector. Kết quả này không đo hiệu quả một perception system độc lập với GT.

`grade_run.log` là JSONL, mỗi `(mode,frame)` đúng một record với các trường:
`mode`, `frame`, `det_tp`, `det_fp`, `det_fn`, `valid_gt`, `confirmed`, `matches`,
`sum_sq_err`, `ghosts`, `misses`. Đảm bảo `matches+ghosts==confirmed` và
`matches+misses==valid_gt`; tổng/trung bình record phải khớp `metrics.json`.
File per-mode `metrics_lidar.json`, `metrics_fused.json`, `grade_run_lidar.log`,
`grade_run_fused.log` được giữ để đối chiếu.

## Giải thích ngắn (Parts E–H — tự viết)

1. Khác biệt đo lidar 3D và camera 2D trong EKF (`z`, `R`)?
   LiDAR đo trực tiếp vị trí 3D trong không gian xe: vector đo z thuộc R^3 là (x, y, z) tính bằng mét; ma trận hiệp phương sai R thuộc R^(3x3) là ma trận đường chéo với các độ lệch chuẩn sigma_lidar_x,y,z = 0.1 m (phương sai 0.01 m^2); ma trận đo H là tuyến tính chiếu 3D sang 3D. Ngược lại, Camera đo tọa độ điểm ảnh 2D trên mặt phẳng ảnh: vector đo z thuộc R^2 là (u, v) tính bằng pixel; ma trận hiệp phương sai R thuộc R^(2x2) là ma trận đường chéo với sigma_cam_i,j = 5 pixel (phương sai 25 pixel^2); hàm đo h(x) là phi tuyến theo mô hình pinhole chia phối cảnh cho độ sâu x_s, nên ma trận quan sát H thuộc R^(2x6) là ma trận Jacobian đạo hàm chuỗi (chain rule).

2. Vì sao cần gating Mahalanobis trước khi gán?
   Khoảng cách Mahalanobis d^2 = gamma^T * S^(-1) * gamma chuẩn hóa độ lệch innovation gamma theo hiệp phương sai S = H*P*H^T + R, phản ánh mức độ tương thích thống kê giữa đo lường và dự báo track. Gating qua phân phối Chi-bình phương (chi^2) giúp loại bỏ các đo lường nhiễu ngoại lai (clutter / false alarms) hoặc các phép đo thuộc về đối tượng khác nằm ngoài vùng tin cậy (ngưỡng 99.5%). Điều này ngăn chặn việc gán nhầm làm phân kỳ bộ lọc EKF và giảm thiểu không gian tìm kiếm trước khi thực hiện greedy assignment.

3. Pipeline là track-then-fuse hay fuse-then-track? Chỉ ra trên log `fusion-run-lab`.
   Pipeline là track-then-fuse. Hệ thống không gộp dữ liệu cảm biến trước khi phát hiện (không kết hợp raw point cloud và pixel), mà duy trì một danh sách track duy nhất. Trong mỗi frame Waymo, EKF thực hiện dự báo (predict) một lần duy nhất cho mọi track, sau đó lần lượt gán và cập nhật EKF qua từng modality: trước tiên là lượt LiDAR (AssocL), tiếp sau đó là lượt Camera (AssocC). Trên log của fusion-run-lab, thứ tự hiển thị rõ: mỗi frame luôn có bước EKF predict -> AssocL lidar update -> quản lý track -> AssocC camera update trên cùng các track đó (không predict lại giữa hai cảm biến).

4. Nếu camera lệch calibration, triệu chứng gì trên innovation/residual?
   Nếu camera bị lệch calibration (lệch extrinsic xoay/tịnh tiến hoặc intrinsic tiêu cự/tâm ảnh), hàm dự báo h(x) sẽ tính ra tọa độ pixel bị lệch một khoảng có hệ thống (systematic bias). Dẫn tới innovation gamma = z - h(x) sẽ có kỳ vọng khác 0 (non-zero mean residual) thay vì là nhiễu trắng Gaussian kỳ vọng 0. Khoảng cách Mahalanobis d^2 sẽ tăng cao bất thường khiến các đo lường camera dễ bị loại bởi cổng chi^2, hoặc nếu lọt qua gate sẽ làm ước lượng vị trí bị kéo lệch và tăng hiệp phương sai P, làm giảm độ chính xác tracking (tăng RMSE).

5. Vì sao `associate_and_update(..., sensor)` cần sensor tường minh ở frame rỗng?
   Giải thích vì sao lidar quyết định score/init/delete còn camera chỉ EKF update.
   Khi meas_list rỗng (không có vật thể nào trong frame), associate_and_update vẫn bắt buộc phải gọi manager.manage_tracks(unassigned_tracks, unassigned_meas, sensor) để xử lý vòng đời. Tham số sensor tường minh giúp manager nhận biết lượt hiện tại là của LiDAR (để trừ điểm các track miss trong FOV và xóa track nếu cạn điểm) hay của Camera (bỏ qua không đổi lifecycle).
   LiDAR quyết định score/init/delete vì LiDAR cung cấp tọa độ 3D đầy đủ và bao quát không gian xung quanh xe, đủ thông tin để định vị khởi tạo xe mới và đánh giá độ tin cậy tồn tại (hit cộng điểm, miss trừ điểm). Camera chỉ thực hiện EKF update vì camera chỉ đo 2D góc chiếu phối cảnh (mất chiều sâu, không thể tự sinh track 3D độc lập), tầm nhìn giới hạn ở phía trước và dễ bị che khuất; nếu camera tham gia xóa track thì các xe chạy ra ngoài góc nhìn camera trước nhưng vẫn trong tầm LiDAR sẽ bị xóa nhầm.

6. Nêu điều kiện xác nhận, giữ confirmed sau miss, và điều kiện xóa track.
   - Điều kiện xác nhận (confirmed): Khi score > confirmed_threshold (0.8). Mỗi lần hit LiDAR cộng thêm 1/window (1/6); sau 5 hit liên tiếp (score = 5/6 = 0.833 > 0.8), track chuyển sang confirmed.
   - Giữ confirmed sau miss: Nếu track đã đạt trạng thái confirmed, khi gặp miss trong LiDAR FOV, điểm số bị trừ 1/window nhưng trạng thái vẫn được bảo toàn là confirmed (không bị hạ về tentative hay initialized), tránh chập chờn danh tính do cảm biến chập chờn tức thời.
   - Điều kiện xóa track (should_delete_track): Track bị xóa nếu thỏa mãn bất kỳ điều kiện nào (toán tử OR): (1) Hiệp phương sai vị trí P[0,0] > max_P hoặc P[1,1] > max_P (với max_P = 9.0 m^2); (2) Track đã confirmed nhưng score < delete_threshold (0.6); (3) Track chưa confirmed (initialized/tentative) nhưng score <= 0.0.

## Bonus (không bắt buộc)

Liệt kê phần bonus đã làm, file bằng chứng trong `student/bonus/` và kết quả chính
(xem [RUBRIC.md](../RUBRIC.md) mục 2). Không làm thì ghi "Không".

- Không

## Khai báo sử dụng AI (bắt buộc)

Ghi rõ, kể cả khi không dùng ("Không dùng AI"). Xem [RULES.md](../RULES.md) mục 2.

- Công cụ đã dùng (ChatGPT, Copilot, Claude, …): Antigravity (Gemini 3.8 Flash)
- Dùng cho phần nào (hàm, câu hỏi, debug): Hỗ trợ lập trình 4 module Part E-H (kalman, camera_fusion, association, track_management), giải thích 6 câu hỏi kỹ thuật chuyên sâu về Sensor Fusion, hỗ trợ chạy kiểm thử và thiết lập môi trường.
- Cách bạn đã kiểm tra lại (pytest, chạy Waymo, đối chiếu công thức): Chạy toàn bộ 128 unit test (`pytest student/tests`) đạt 100% pass, chạy mô phỏng Waymo full segment 199 frame (`fusion-run-lab --fusion compare --seed 0`) xác minh RMSE đạt 0.1358 m <= 0.45 m chuẩn, đối chiếu công thức ma trận EKF và kiểm tra hợp lệ bằng `tools/check_submission.py`.

## Checklist nộp

- [x] **Part E–H** trong `workspace/` đã implement; `pytest student/tests -q` không còn `failed`/`xfailed`
- [x] Part A–D: không bắt buộc sửa (hoặc ghi chú nếu bạn đã sửa)
- [x] Lần chạy chấm điểm: `--fusion compare --seed 0`, `frame_start: 0`, `frame_end: 198`
- [x] Đã commit `student/artifacts/metrics*.json` và `student/artifacts/grade_run*.log` (không sửa tay)
- [x] Đã điền đủ file này, gồm khai báo AI
- [x] Không commit dữ liệu Waymo, weights, `paths.yaml`, API key
- [x] `python tools/check_submission.py` báo `KẾT QUẢ: SẴN SÀNG NỘP`
- [x] Đã push và nộp link repo + commit hash trên LMS ([hướng dẫn nộp](../SUBMISSION.md))
