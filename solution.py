"""Student entry point. See README.md for the API and scoring rules."""

def solve(initial_state, timebound=120):
    """Return a goal SokobanState with a valid parent chain, or False."""
    raise NotImplementedError("Implement your Sokoban solver")


def heuristic(state, dist_table=None):
    total_distance = 0

    # Duyệt qua từng cái thùng trên bàn cờ
    for box in state.boxes:

        # Tìm khoảng cách thực tế nhỏ nhất từ thùng này tới một ô đích bất kỳ
        min_dist = float('inf')  # Khởi đầu lớn

        for target in state.storage:
            # Tra cứu: Ô 'box' cách 'target' bao nhiêu bước
            # Nếu trong sổ không có (nghĩa là bị tường bịt kín không tới được), coi như vô cùng
            d = dist_table[target].get(box, float('inf'))
            if d < min_dist:
                min_dist = d

        # Nếu thùng này không thể tới BẤT KỲ ô đích nào
        if min_dist == float('inf'):
            # Trạng thái này CHẮC CHẮN THUA (thùng bị cô lập trong hốc kẹt)
            # Trả về vô cùng để A* vứt bỏ ngay nhánh này, không cần duyệt tiếp
            return float('inf')

        total_distance += min_dist

    return total_distance



from collections import deque

def precompute(initial_state):
    """
    Tính trước bảng khoảng cách thực tế (tránh tường) từ mỗi ô đích ra khắp bản đồ.
    Chỉ chạy đúng 1 LẦN DUY NHẤT trước khi bắt đầu giải!
    """
    width = initial_state.width
    height = initial_state.height
    obstacles = initial_state.obstacles  # Tập hợp các ô tường
    storage = initial_state.storage      # Tập hợp các ô đích

    # Cuốn sổ lưu kết quả
    dist_table = {}

    # 1. Duyệt qua từng ô đích (mỗi ô đích mở 1 "vòi nước")
    for target in storage:
        # Sổ con ghi khoảng cách từ đích 'target' tới mọi ô: {(x, y): số_bước}
        distances = {}

        # Hàng đợi BFS: chứa các cặp (tọa_độ, số_bước)
        # Bắt đầu ngay tại đích với 0 bước
        queue = deque([(target, 0)])
        distances[target] = 0

        # 2. Bắt đầu cho nước loang
        while queue:
            # Lấy ô đầu hàng đợi ra xét
            (curr_x, curr_y), d = queue.popleft()

            # Thử loang ra 4 hướng: Lên, Xuống, Trái, Phải
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nx = curr_x + dx
                ny = curr_y + dy

                # Kiểm tra 1: Ô mới có nằm trong bàn cờ không
                if 0 <= nx < width and 0 <= ny < height:

                    # Kiểm tra 2: Ô mới có phải là tường không
                    if (nx, ny) not in obstacles:

                        # Kiểm tra 3: Nước đã chảy tới ô này chưa
                        if (nx, ny) not in distances:
                            # Ghi nhận ô mới cách đích d + 1 bước
                            distances[(nx, ny)] = d + 1
                            # Đưa ô mới vào hàng đợi để loang tiếp
                            queue.append(((nx, ny), d + 1))

        # Lưu sổ con của đích này vào cuốn sổ lớn
        dist_table[target] = distances

    return {"dist_table": dist_table}