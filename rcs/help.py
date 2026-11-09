"""What each page does, for the "?" button (and Shift+F1): the page on screen,
in the app's language. The full guide is docs/USER_GUIDE.md."""
# flake8: noqa: E501
from __future__ import annotations

GUIDE = "https://github.com/Tbm615/AMR-RCS/blob/main/docs/USER_GUIDE.md"

EN = {
    "conn": ("Connection", """
<p>Your robots and how the app reaches them.</p>
<ul>
<li><b>Robots</b>: pick one, <b>+ Add</b> or <b>Remove</b>. Each robot keeps its own missions, places, events and settings.</li>
<li><b>Connection</b>: the robot's address (Host, Port 9090). Tick <b>wss://</b> if its gateway has a certificate.</li>
<li><b>Sign-in</b>: robots with the gateway ask who you are. Your role decides what you may do: viewer watches and may stop the robot, operator drives, admin also manages users (<b>Users and audit log…</b>).</li>
<li><b>Topics / Frames</b>: change only if the robot's launch files differ.</li>
<li><b>Limits</b>: top speeds and the <b>Footprint</b> used by the clearance bars.</li>
<li><b>Robot model</b>: how the robot is drawn (Hinson, a box, or a URDF/mesh you import).</li>
<li><b>Save changes</b>, then <b>Connect</b>. Below: connect at start, new-version notice, <b>Language</b>.</li>
</ul>"""),
    "robot": ("Robot programs", """
<p>Start and stop the robot's programs without a terminal.</p>
<ol>
<li><b>Robot base</b> → <b>Start</b>: drives, lidar, camera.</li>
<li><b>Navigation</b>: on a <b>saved map</b> (then set the initial pose on the 2D Map), or <b>SLAM</b> to build a new map while driving.</li>
<li><b>Save the map</b> when SLAM has covered the area; it then appears under saved maps.</li>
</ol>
<p><b>Show log</b> says why a program failed. The banner says if the robot launcher is not running or its version is too old.</p>"""),
    "status": ("Status", """
<ul>
<li><b>Health</b>: one line per part of the robot, green, amber or red, with what to do.</li>
<li><b>Save diagnostic bundle…</b>: one zip for whoever helps you (no passwords or tokens in it).</li>
<li>Speeds, E-STOP, and the rate of every topic: <b>—</b> means nothing arrives.</li>
</ul>"""),
    "teleop": ("Teleop", """
<ul>
<li>Drag the <b>joystick</b>: up forward, sideways turn; let go to stop.</li>
<li>Keys (click the page first): <b>W A S D</b> or arrows, <b>Space</b> stop, <b>1 2 3</b> speed.</li>
<li><b>Gamepad</b> on this PC: hold LB and use the sticks; B is E-STOP.</li>
<li><b>Clearance</b> bars: distance from the body to the nearest obstacle. Display only: the app does not brake for you.</li>
<li>The app sends only while you drive, so Nav2 is never blocked.</li>
</ul>"""),
    "map": ("2D Map", """
<ul>
<li>The map, the lidar points (red), the planned path (green) and the robot seen from above.</li>
<li><b>Press and drag</b> on the map: an arrow follows the mouse; release to send. Choose what it sets in <b>Tools</b>: goal, initial pose, waypoint, or measure a distance.</li>
<li><b>Use last position</b>: the initial pose where the robot stopped last time on this map.</li>
<li>Wheel zooms, right-drag pans; <b>Center on robot</b> / <b>Whole map</b>. <b>Overlays</b>: costmaps, places, waypoints.</li>
</ul>"""),
    "nav": ("Navigation", """
<ul>
<li>Nav2's state of the current goal, distance and time left, <b>Cancel</b>.</li>
<li><b>Saved places</b>: <b>Save robot position</b>, then <b>Go</b> to drive back there, or <b>→ Mission</b> to add it to the open mission.</li>
</ul>"""),
    "mission": ("Missions", """
<ul>
<li>A mission is waypoints in order, with a tolerance and a wait at each. Add them on the map (<b>Add waypoint</b> tool) or <b>Set here</b> at the robot.</li>
<li><b>Loop route</b> and <b>Laps</b> repeat it; select rows to move, remove or <b>Apply to selected</b>.</li>
<li><b>▶ Play</b>, <b>⏸ Pause</b>, <b>■ Stop</b>; <b>Start from selected waypoint</b> after an interruption.</li>
<li>A goal Nav2 drops is retried. A mission belongs to the map it was made on.</li>
</ul>"""),
    "3d": ("3D Map", """
<ul>
<li>A colored 3D map built from the camera while this page is open (<b>Record 3D map</b>).</li>
<li><b>View</b>: fusion with the 2D layers, 3D only, or top view. Drag to orbit, wheel to zoom.</li>
<li>With a mouse mode other than orbit, press on the floor and drag toward the heading to send a goal, a pose or a waypoint.</li>
<li><b>Save PLY…</b> / <b>Open PLY…</b>; raise the <b>Voxel</b> size if the map gets full.</li>
</ul>"""),
    "cam": ("Camera", """
<p>The robot's camera, received only while this page is open to spare Wi-Fi. If there is no picture, the page says why (camera not started, USB problem...).</p>"""),
    "logs": ("Logs", """
<p>The robot's ROS messages (/rosout). Choose a minimum level, auto-scroll, or clear.</p>"""),
    "events": ("Events", """
<ul>
<li>What happened and when: connections, goals, missions, E-STOP, battery, errors. Filter by level or source, <b>Export CSV…</b>.</li>
<li><b>Sound alerts</b> on errors and warnings; <b>Remote alerts…</b> sends them to Telegram or e-mail.</li>
</ul>"""),
    "data": ("Data", """
<ul>
<li><b>Back up…</b> everything about this robot in one file, or <b>Share…</b> missions, places and maps with others; <b>Import…</b> only adds.</li>
<li>Copy maps between this PC and the robot: <b>← Copy to this PC</b>, <b>Send to robot →</b>.</li>
</ul>"""),
    "battery": ("Battery", """
<ul>
<li><b>Now</b>: charge, voltage, current, temperatures and every cell, from the robot's BMS.</li>
<li><b>History</b>: a log per boot on the robot; charts per boot or over many boots, and how each boot ended.</li>
<li><b>Charging</b>: go to charge by itself below a charge, at a saved place, and carry on the mission when charged.</li>
</ul>"""),
    "schedule": ("Schedule", """
<ul>
<li>Missions that start by themselves at set times and days, if the robot is ready (connected, battery, no E-STOP).</li>
<li><b>Run reports</b>: every run, how it ended and why; export as CSV.</li>
<li><b>Statistics</b>: a period's missions, success, distance and incidents; <b>Save PDF…</b>.</li>
</ul>"""),
    "fleet": ("Fleet", """
<ul>
<li>Every robot as a card: tick <b>Watch</b> to follow it (a light connection each).</li>
<li>The robot drawn from above, turned the way it faces, colored by its state.</li>
<li><b>Open</b> connects to drive it; <b>E-STOP</b> or <b>E-STOP all watched</b> only engages (release it on that robot's own pages).</li>
</ul>"""),
}

VI = {
    "conn": ("Kết nối", """
<p>Danh sách robot và cách app kết nối tới chúng.</p>
<ul>
<li><b>Robot</b>: chọn một robot, <b>+ Thêm</b> hoặc <b>Xoá</b>. Mỗi robot có nhiệm vụ, điểm đã lưu, sự kiện và cài đặt riêng.</li>
<li><b>Kết nối</b>: địa chỉ robot (Địa chỉ, Cổng 9090). Tick <b>wss://</b> nếu cổng an ninh của robot có chứng chỉ.</li>
<li><b>Đăng nhập</b>: robot có cổng an ninh sẽ hỏi bạn là ai. Vai trò quyết định bạn được làm gì: viewer chỉ xem và được dừng robot, operator được lái, admin quản lý thêm người dùng (<b>Người dùng và nhật ký thao tác…</b>).</li>
<li><b>Topic / Frame</b>: chỉ đổi nếu launch file của robot khác mặc định.</li>
<li><b>Giới hạn</b>: tốc độ tối đa và <b>Kích thước</b> dùng cho thanh khoảng trống.</li>
<li><b>Model robot</b>: cách vẽ robot (Hinson, hộp, hoặc URDF/mesh bạn nhập).</li>
<li><b>Lưu thay đổi</b>, rồi <b>Kết nối</b>. Bên dưới: tự kết nối khi mở app, báo bản mới, <b>Ngôn ngữ</b>.</li>
</ul>"""),
    "robot": ("Chương trình robot", """
<p>Bật, tắt các chương trình trên robot mà không cần terminal.</p>
<ol>
<li><b>Thân robot</b> → <b>Bật</b>: động cơ, lidar, camera.</li>
<li><b>Dẫn đường</b>: trên <b>bản đồ đã lưu</b> (sau đó đặt vị trí ban đầu ở Bản đồ 2D), hoặc <b>SLAM</b> để lập bản đồ mới khi chạy.</li>
<li><b>Lưu bản đồ</b> khi SLAM đã quét hết khu vực; bản đồ hiện trong danh sách đã lưu.</li>
</ol>
<p><b>Xem log</b> cho biết vì sao chương trình lỗi. Dòng trên cùng báo nếu trình khởi chạy chưa chạy hoặc phiên bản phía robot quá cũ.</p>"""),
    "status": ("Trạng thái", """
<ul>
<li><b>Sức khoẻ</b>: mỗi bộ phận một dòng, xanh, vàng hoặc đỏ, kèm cách xử lý.</li>
<li><b>Lưu gói chẩn đoán…</b>: một file zip gửi người hỗ trợ (không chứa mật khẩu hay token).</li>
<li>Tốc độ, dừng khẩn, và tần số mỗi topic: <b>—</b> nghĩa là không nhận được gì.</li>
</ul>"""),
    "teleop": ("Lái tay", """
<ul>
<li>Kéo <b>cần điều khiển</b>: lên là tiến, sang ngang là quay; thả ra để dừng.</li>
<li>Phím (bấm vào trang trước): <b>W A S D</b> hoặc mũi tên, <b>Space</b> dừng, <b>1 2 3</b> đổi tốc độ.</li>
<li><b>Tay cầm</b> trên máy này: giữ LB và dùng cần; B là dừng khẩn.</li>
<li>Thanh <b>khoảng trống</b>: khoảng cách từ thân robot tới vật cản gần nhất. Chỉ để xem: app không phanh thay bạn.</li>
<li>App chỉ gửi lệnh khi bạn đang lái, nên không bao giờ chặn Nav2.</li>
</ul>"""),
    "map": ("Bản đồ 2D", """
<ul>
<li>Bản đồ, điểm lidar (đỏ), đường đi dự kiến (xanh lá) và robot nhìn từ trên xuống.</li>
<li><b>Nhấn và kéo</b> trên bản đồ: mũi tên theo chuột, thả ra để gửi. Chọn việc nó làm trong <b>Công cụ</b>: đặt đích, vị trí ban đầu, điểm dừng, hoặc đo khoảng cách.</li>
<li><b>Dùng vị trí cuối</b>: đặt vị trí ban đầu tại chỗ robot dừng lần trước trên bản đồ này.</li>
<li>Lăn chuột để phóng to, kéo chuột phải để di chuyển; <b>Về robot</b> / <b>Toàn bản đồ</b>. <b>Lớp hiển thị</b>: costmap, điểm đã lưu, điểm dừng.</li>
</ul>"""),
    "nav": ("Dẫn đường", """
<ul>
<li>Trạng thái Nav2 của đích hiện tại, quãng đường và thời gian còn lại, <b>Huỷ</b>.</li>
<li><b>Điểm đã lưu</b>: <b>Lưu vị trí robot</b>, rồi <b>Đi</b> để quay lại đó, hoặc <b>→ Nhiệm vụ</b> để thêm vào nhiệm vụ đang mở.</li>
</ul>"""),
    "mission": ("Nhiệm vụ", """
<ul>
<li>Nhiệm vụ là các điểm dừng theo thứ tự, mỗi điểm có sai số và thời gian chờ. Thêm điểm trên bản đồ (công cụ <b>Thêm điểm dừng</b>) hoặc <b>Đặt ở đây</b> tại vị trí robot.</li>
<li><b>Lặp lộ trình</b> và <b>Số vòng</b> để chạy lặp; chọn các dòng để di chuyển, xoá hoặc <b>Áp cho các điểm đã chọn</b>.</li>
<li><b>▶ Chạy</b>, <b>⏸ Tạm dừng</b>, <b>■ Dừng</b>; <b>Chạy từ điểm đã chọn</b> sau khi bị gián đoạn.</li>
<li>Đích bị Nav2 bỏ sẽ được gửi lại. Nhiệm vụ gắn với bản đồ nó được tạo.</li>
</ul>"""),
    "3d": ("Bản đồ 3D", """
<ul>
<li>Bản đồ 3D có màu dựng từ camera khi trang này đang mở (<b>Ghi bản đồ 3D</b>).</li>
<li><b>Kiểu xem</b>: kết hợp với các lớp 2D, chỉ 3D, hoặc nhìn từ trên. Kéo để xoay, lăn chuột để phóng to.</li>
<li>Ở chế độ chuột khác xoay, nhấn trên sàn và kéo theo hướng để gửi đích, vị trí hoặc điểm dừng.</li>
<li><b>Lưu PLY…</b> / <b>Mở PLY…</b>; tăng <b>Voxel</b> nếu bản đồ đầy.</li>
</ul>"""),
    "cam": ("Camera", """
<p>Camera của robot, chỉ nhận khi trang này mở để tiết kiệm Wi-Fi. Nếu không có hình, trang sẽ nói lý do (camera chưa bật, lỗi USB...).</p>"""),
    "logs": ("Nhật ký ROS", """
<p>Thông điệp ROS của robot (/rosout). Chọn mức tối thiểu, tự cuộn, hoặc xoá.</p>"""),
    "events": ("Sự kiện", """
<ul>
<li>Việc gì xảy ra và khi nào: kết nối, đích, nhiệm vụ, dừng khẩn, pin, lỗi. Lọc theo mức hoặc nguồn, <b>Xuất CSV…</b>.</li>
<li><b>Âm báo</b> khi có lỗi và cảnh báo; <b>Cảnh báo từ xa…</b> gửi qua Telegram hoặc e-mail.</li>
</ul>"""),
    "data": ("Dữ liệu", """
<ul>
<li><b>Sao lưu…</b> mọi thứ của robot này vào một file, hoặc <b>Chia sẻ…</b> nhiệm vụ, điểm và bản đồ cho người khác; <b>Nhập…</b> chỉ thêm, không thay thế.</li>
<li>Chép bản đồ giữa máy này và robot: <b>← Chép về máy này</b>, <b>Gửi lên robot →</b>.</li>
</ul>"""),
    "battery": ("Pin", """
<ul>
<li><b>Hiện tại</b>: dung lượng, điện áp, dòng, nhiệt độ và từng cell, từ BMS của robot.</li>
<li><b>Lịch sử</b>: mỗi lần bật robot một file log; biểu đồ từng lần bật hoặc nhiều lần, và mỗi lần kết thúc thế nào.</li>
<li><b>Sạc</b>: tự đi sạc khi pin dưới ngưỡng, tới điểm đã lưu, sạc xong chạy tiếp nhiệm vụ.</li>
</ul>"""),
    "schedule": ("Lịch chạy", """
<ul>
<li>Nhiệm vụ tự chạy vào giờ và ngày đã đặt, nếu robot sẵn sàng (đã kết nối, đủ pin, không dừng khẩn).</li>
<li><b>Báo cáo các lần chạy</b>: mỗi lần chạy, kết thúc thế nào và vì sao; xuất CSV.</li>
<li><b>Thống kê</b>: nhiệm vụ, tỉ lệ thành công, quãng đường và sự cố trong một khoảng thời gian; <b>Lưu PDF…</b>.</li>
</ul>"""),
    "fleet": ("Đội robot", """
<ul>
<li>Mỗi robot một thẻ: tick <b>Theo dõi</b> để theo dõi (mỗi robot một kết nối nhẹ).</li>
<li>Robot được vẽ từ trên xuống, xoay theo hướng nó đang quay, tô màu theo trạng thái.</li>
<li><b>Mở</b> để kết nối và lái; <b>Dừng khẩn</b> hoặc <b>Dừng khẩn mọi robot</b> chỉ bật (nhả trên trang của robot đó).</li>
</ul>"""),
}

COMMON = {
    "en": ("<hr><p><b>Anywhere:</b> the red <b>E-STOP</b> (or F12) stops the robot; <b>Layout</b> shows several pages at "
           "once; F1–F11 open the pages; Esc cancels the goal or pauses the mission.</p>"
           f'<p>Full guide: <a href="{GUIDE}" style="color:#4fd1ff;">User guide</a></p>'),
    "vi": ("<hr><p><b>Mọi lúc:</b> nút đỏ <b>DỪNG KHẨN</b> (hoặc F12) dừng robot; <b>Bố cục</b> hiện nhiều trang cùng lúc; "
           "F1–F11 mở các trang; Esc huỷ đích hoặc tạm dừng nhiệm vụ.</p>"
           f'<p>Hướng dẫn đầy đủ: <a href="{GUIDE}" style="color:#4fd1ff;">User guide</a></p>'),
}


def page_help(key: str, lang: str = "en"):
    """(title, html) for a page, in the language (English if none)."""
    table = VI if lang == "vi" else EN
    title, body = table.get(key) or EN.get(key) or (key, "<p>—</p>")
    return title, f"<h2>{title}</h2>{body}{COMMON.get(lang, COMMON['en'])}"
