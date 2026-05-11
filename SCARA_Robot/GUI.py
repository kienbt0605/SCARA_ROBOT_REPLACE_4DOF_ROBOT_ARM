import tkinter as tk
from tkinter import ttk
from tkinter import font as tkfont
import tkinter.messagebox as messagebox
import serial
import serial.tools.list_ports

# ==========================================
# BIẾN TOÀN CỤC & HÀNG ĐỢI LỆNH
# ==========================================
arduino = None
cmd_queue = []       # Hàng đợi chứa các lệnh chờ gửi
is_busy = False      # Trạng thái xem Arduino có đang chạy lệnh không
serial_buffer = ""   # Bộ đệm để ghép các mảnh dữ liệu Serial bị đứt đoạn

# ==========================================
# BỘ TỪ ĐIỂN FONT CHỮ (VECTOR FONT)
# ==========================================
VECTOR_FONT = {
    'A': [(0,0, 0.5,1), (0.5,1, 1,0), (0.2,0.5, 0.8,0.5)],
    'B': [(0,0, 0,1), (0,1, 0.8,1), (0.8,1, 1,0.75), (1,0.75, 0.8,0.5), (0.8,0.5, 0,0.5), (0.8,0.5, 1,0.25), (1,0.25, 0.8,0), (0.8,0, 0,0)],
    'C': [(1,0.2, 0.8,0), (0.8,0, 0.2,0), (0.2,0, 0,0.2), (0,0.2, 0,0.8), (0,0.8, 0.2,1), (0.2,1, 0.8,1), (0.8,1, 1,0.8)],
    'D': [(0,0, 0,1), (0,1, 0.8,1), (0.8,1, 1,0.8), (1,0.8, 1,0.2), (1,0.2, 0.8,0), (0.8,0, 0,0)],
    'E': [(1,0, 0,0), (0,0, 0,1), (0,1, 1,1), (0,0.5, 0.8,0.5)],
    'F': [(0,0, 0,1), (0,1, 1,1), (0,0.5, 0.8,0.5)],
    'G': [(1,1, 0,1), (0,1, 0,0), (0,0, 1,0), (1,0, 1,0.5), (0.5,0.5, 1,0.5)],
    'H': [(0,0, 0,1), (1,0, 1,1), (0,0.5, 1,0.5)],
    'I': [(0.5,0, 0.5,1), (0.25,0, 0.75,0), (0.25,1, 0.75,1)],
    'J': [(0,0.5, 0.5,0), (0.5,0, 1,0), (1,0, 1,1)],
    'K': [(0,0, 0,1), (1,1, 0,0.5), (0,0.5, 1,0)],
    'L': [(0,1, 0,0), (0,0, 1,0)],
    'M': [(0,0, 0,1), (0,1, 0.5,0.5), (0.5,0.5, 1,1), (1,1, 1,0)],
    'N': [(0,0, 0,1), (0,1, 1,0), (1,0, 1,1)],
    'O': [(0,0, 0,1), (0,1, 1,1), (1,1, 1,0), (1,0, 0,0)],
    'P': [(0,0, 0,1), (0,1, 1,1), (1,1, 1,0.5), (1,0.5, 0,0.5)],
    'Q': [(0,0, 0,1), (0,1, 1,1), (1,1, 1,0), (1,0, 0,0), (0.5,0.5, 1,-0.2)],
    'R': [(0,0, 0,1), (0,1, 1,1), (1,1, 1,0.5), (1,0.5, 0,0.5), (0,0.5, 1,0)],
    'S': [(0,0, 1,0), (1,0, 1,0.5), (1,0.5, 0,0.5), (0,0.5, 0,1), (0,1, 1,1)],
    'T': [(0,1, 1,1), (0.5,1, 0.5,0)],
    'U': [(0,1, 0,0), (0,0, 1,0), (1,0, 1,1)],
    'V': [(0,1, 0.5,0), (0.5,0, 1,1)],
    'W': [(0,1, 0.25,0), (0.25,0, 0.5,0.5), (0.5,0.5, 0.75,0), (0.75,0, 1,1)],
    'X': [(0,0, 1,1), (0,1, 1,0)],
    'Y': [(0,1, 0.5,0.5), (1,1, 0.5,0.5), (0.5,0.5, 0.5,0)],
    'Z': [(0,1, 1,1), (1,1, 0,0), (0,0, 1,0)],
    '0': [(0,0, 0,1), (0,1, 1,1), (1,1, 1,0), (1,0, 0,0), (0,0, 1,1)],
    '1': [(0.2,0.8, 0.5,1), (0.5,1, 0.5,0), (0.2,0, 0.8,0)],
    '2': [(0,1, 1,1), (1,1, 1,0.5), (1,0.5, 0,0.5), (0,0.5, 0,0), (0,0, 1,0)],
    '3': [(0,1, 1,1), (1,1, 1,0), (1,0, 0,0), (0,0.5, 1,0.5)],
    '4': [(0,1, 0,0.5), (0,0.5, 1,0.5), (1,1, 1,0)],
    '5': [(1,1, 0,1), (0,1, 0,0.5), (0,0.5, 1,0.5), (1,0.5, 1,0), (1,0, 0,0)],
    '6': [(1,1, 0,1), (0,1, 0,0), (0,0, 1,0), (1,0, 1,0.5), (1,0.5, 0,0.5)],
    '7': [(0,1, 1,1), (1,1, 0.5,0)],
    '8': [(0,0, 0,1), (0,1, 1,1), (1,1, 1,0), (1,0, 0,0), (0,0.5, 1,0.5)],
    '9': [(0,0, 1,0), (1,0, 1,1), (1,1, 0,1), (0,1, 0,0.5), (0,0.5, 1,0.5)],
    '-': [(0.2,0.5, 0.8,0.5)]
}

def create_ui_v2():
    global arduino, is_busy, serial_buffer
    root = tk.Tk()
    root.title("Điều Khiển Robot SCARA - Phiên bản Tiếng Việt")
    root.geometry("1150x900")
    root.configure(bg="#F2F2F2")
    root.resizable(False, False)

    # --- KHAI BÁO FONT & MÀU SẮC ---
    font_title = tkfont.Font(family="Arial", size=22, weight="bold")
    font_subtitle = tkfont.Font(family="Arial", size=15, weight="bold")
    font_status = tkfont.Font(family="Arial", size=16, weight="bold")
    font_label = tkfont.Font(family="Arial", size=11)
    font_btn = tkfont.Font(family="Arial", size=10, weight="bold")
    
    BG_COLOR = "#F2F2F2"
    BTN_COLOR = "#0C2E59"
    BTN_TEXT = "#FFFFFF"

    # --- HÀM GỬI LỆNH (CÓ HÀNG ĐỢI) ---
    def send_command(cmd, force=False):
        global arduino, is_busy
        if arduino and arduino.is_open:
            try:
                if force:
                    arduino.write((cmd + "\n").encode('utf-8'))
                    is_busy = True
                    print(f"Đã gửi (Ép buộc): {cmd}")
                elif not is_busy:
                    arduino.write((cmd + "\n").encode('utf-8'))
                    is_busy = True
                    print(f"Đã gửi: {cmd}")
                else:
                    cmd_queue.append(cmd)
                    lbl_queue.config(text=f"Lệnh chờ: {len(cmd_queue)}")
            except Exception as e:
                print("Lỗi viết Serial:", e)
        else:
            messagebox.showwarning("Chưa kết nối", "Vui lòng kết nối Arduino trước khi gửi lệnh!")

    # --- HÀM ĐỌC DỮ LIỆU TỪ ARDUINO (ĐÃ SỬA LỖI ĐỨT MẠCH SERIAL) ---
    def read_serial():
        global arduino, is_busy, serial_buffer
        if arduino and arduino.is_open:
            try:
                # 1. Đọc và gom tất cả dữ liệu đang có trong cổng vào bộ đệm
                while arduino.in_waiting > 0:
                    serial_buffer += arduino.read(arduino.in_waiting).decode('utf-8', errors='ignore')
                
                # 2. Xử lý từng dòng nếu đã có dấu xuống dòng (\n)
                if '\n' in serial_buffer:
                    lines = serial_buffer.split('\n')
                    serial_buffer = lines[-1] # Phần dư thừa chưa trọn vẹn thì để lại cho vòng lặp sau
                    
                    for line in lines[:-1]:
                        data = line.strip()
                        
                        if "STATUS," in data:
                            # Cắt lấy đúng phần thông số sau chữ STATUS,
                            status_str = data.split("STATUS,")[1]
                            parts = status_str.split(',')
                            if len(parts) == 5:
                                x, y, z, j1, j2 = parts
                                lbl_x.config(text=f"X: {float(x):.1f} mm")
                                lbl_y.config(text=f"Y: {float(y):.1f} mm")
                                lbl_z.config(text=f"Z: {float(z):.1f} mm")
                                lbl_j1.config(text=f"J1: {float(j1):.1f}°")
                                lbl_j2.config(text=f"J2: {float(j2):.1f}°")
                            
                            # CỰC KỲ QUAN TRỌNG: Nhận STATUS tức là Arduino đã làm xong lệnh cũ
                            is_busy = False
                            
                            # Lôi lệnh tiếp theo trong hàng đợi ra gửi
                            if len(cmd_queue) > 0:
                                next_cmd = cmd_queue.pop(0)
                                lbl_queue.config(text=f"Lệnh chờ: {len(cmd_queue)}")
                                send_command(next_cmd)
                                
                        elif data:
                            print(f"Arduino trả về: {data}")
                            # --- CẢNH BÁO NGUỒN YẾU / RESET ---
                            if "HE THONG DA KHOI DONG" in data:
                                is_busy = False
                                cmd_queue.clear()
                                lbl_queue.config(text="Lệnh chờ: 0")
                                messagebox.showerror("Lỗi Nguồn (Sụt áp)", "Phát hiện Arduino bị khởi động lại đột ngột!\n\nNguyên nhân: Do tốc độ trục Z đang để quá cao, motor rút dòng mạnh làm bo CNC Shield sụt áp.\n\nCách khắc phục: Hủy lệnh vẽ. Hãy mở file Arduino và hạ MaxSpeed/Acceleration của stepperZ xuống (VD: 4000 hoặc 5000).")
            except Exception as e:
                pass
        
        # Cho hàm lặp lại sau 50ms
        root.after(50, read_serial)

    # --- KẾT NỐI SERIAL ---
    top_frame = tk.Frame(root, bg=BG_COLOR)
    top_frame.pack(fill=tk.X, pady=10, padx=30)
    
    tk.Label(top_frame, text="BẢNG ĐIỀU KHIỂN ROBOT SCARA", font=font_title, bg=BG_COLOR, fg="#222222").pack(side=tk.LEFT)
    
    connect_frame = tk.Frame(top_frame, bg=BG_COLOR)
    connect_frame.pack(side=tk.RIGHT)
    tk.Label(connect_frame, text="Cổng COM:", font=font_label, bg=BG_COLOR).pack(side=tk.LEFT)
    
    port_cb = ttk.Combobox(connect_frame, values=[port.device for port in serial.tools.list_ports.comports()], width=10)
    port_cb.pack(side=tk.LEFT, padx=5)
    
    def connect_serial():
        global arduino, serial_buffer
        port = port_cb.get()
        if not port:
            messagebox.showerror("Lỗi", "Vui lòng chọn cổng COM")
            return
        try:
            arduino = serial.Serial(port, 115200, timeout=0.1)
            serial_buffer = ""
            btn_connect.config(text="ĐÃ KẾT NỐI", bg="#27ae60")
            messagebox.showinfo("Thành công", f"Đã kết nối {port}")
            root.after(2000, lambda: send_command("H")) 
        except Exception as e:
            messagebox.showerror("Lỗi Kết Nối", str(e))

    btn_connect = tk.Button(connect_frame, text="KẾT NỐI", font=font_btn, bg="#d35400", fg=BTN_TEXT, command=connect_serial)
    btn_connect.pack(side=tk.LEFT)

    # ==========================================
    # BẢNG TRẠNG THÁI ROBOT
    # ==========================================
    status_frame = tk.LabelFrame(root, text=" Trạng Thái Hiện Tại ", font=font_subtitle, bg=BG_COLOR, fg="#16a085", padx=20, pady=10)
    status_frame.pack(fill=tk.X, padx=30, pady=(0, 20))

    for i in range(5): status_frame.columnconfigure(i, weight=1)

    lbl_x = tk.Label(status_frame, text="X: 0.0 mm", font=font_status, bg=BG_COLOR, fg="#c0392b")
    lbl_x.grid(row=0, column=0)
    lbl_y = tk.Label(status_frame, text="Y: 0.0 mm", font=font_status, bg=BG_COLOR, fg="#c0392b")
    lbl_y.grid(row=0, column=1)
    lbl_z = tk.Label(status_frame, text="Z: 0.0 mm", font=font_status, bg=BG_COLOR, fg="#2980b9")
    lbl_z.grid(row=0, column=2)
    lbl_j1 = tk.Label(status_frame, text="J1: 0.0°", font=font_status, bg=BG_COLOR, fg="#8e44ad")
    lbl_j1.grid(row=0, column=3)
    lbl_j2 = tk.Label(status_frame, text="J2: 0.0°", font=font_status, bg=BG_COLOR, fg="#8e44ad")
    lbl_j2.grid(row=0, column=4)

    lbl_queue = tk.Label(status_frame, text="Lệnh chờ: 0", font=font_label, bg=BG_COLOR, fg="#e67e22")
    lbl_queue.grid(row=1, column=2, pady=5)

    # --- BỐ CỤC ĐIỀU KHIỂN ---
    content_frame = tk.Frame(root, bg=BG_COLOR)
    content_frame.pack(fill=tk.BOTH, expand=True)

    left_frame = tk.Frame(content_frame, bg=BG_COLOR)
    left_frame.pack(side=tk.LEFT, padx=30, fill=tk.Y)
    middle_frame = tk.Frame(content_frame, bg=BG_COLOR)
    middle_frame.pack(side=tk.LEFT, padx=30, fill=tk.Y)
    right_frame = tk.Frame(content_frame, bg=BG_COLOR)
    right_frame.pack(side=tk.LEFT, padx=30, fill=tk.Y)

    # ==========================================
    # CỘT 1: ĐỘNG HỌC THUẬN 
    # ==========================================
    tk.Label(left_frame, text="Động Học Thuận", font=font_subtitle, bg=BG_COLOR, fg="#2980b9").pack(pady=(0, 10))

    jog_frame = tk.LabelFrame(left_frame, text=" Điều khiển từng bước (JOG) ", font=font_label, bg=BG_COLOR, padx=10, pady=10)
    jog_frame.pack(fill=tk.X, pady=10)

    def create_jog_group(parent, label_text, joint_id, unit="độ"):
        frame = tk.Frame(parent, bg=BG_COLOR)
        frame.pack(pady=5)
        tk.Label(frame, text=label_text, font=font_label, bg=BG_COLOR, width=3, anchor="w").grid(row=0, column=0, rowspan=2)
        tk.Label(frame, text=f"Bước ({unit}):", font=tkfont.Font(size=9), bg=BG_COLOR).grid(row=0, column=1, sticky="w")
        
        entry_step = tk.Entry(frame, width=5, justify="center")
        entry_step.insert(0, "5.0")
        entry_step.grid(row=0, column=2, sticky="w")

        def jog(direction):
            try:
                val = float(entry_step.get()) * direction
                send_command(f"J,{joint_id},{val}")
            except ValueError:
                messagebox.showerror("Lỗi", f"Vui lòng nhập số hợp lệ vào ô bước của {label_text}")

        tk.Button(frame, text="- GIẢM", font=font_btn, bg=BTN_COLOR, fg=BTN_TEXT, width=6, command=lambda: jog(-1)).grid(row=1, column=1, padx=5, pady=5)
        tk.Button(frame, text="+ TĂNG", font=font_btn, bg=BTN_COLOR, fg=BTN_TEXT, width=6, command=lambda: jog(1)).grid(row=1, column=2, padx=5, pady=5)

    create_jog_group(jog_frame, "J1", 1, "độ") 
    create_jog_group(jog_frame, "J2", 2, "độ")  
    create_jog_group(jog_frame, "Z", 3, "mm")  

    angle_frame = tk.LabelFrame(left_frame, text=" Nhập góc trực tiếp ", font=font_label, bg=BG_COLOR, padx=10, pady=10)
    angle_frame.pack(fill=tk.X, pady=10)

    tk.Label(angle_frame, text="J1 (Độ):", font=font_label, bg=BG_COLOR).grid(row=0, column=0, pady=5)
    entry_a1 = tk.Entry(angle_frame, width=8, justify="center"); entry_a1.grid(row=0, column=1, padx=5)
    
    tk.Label(angle_frame, text="J2 (Độ):", font=font_label, bg=BG_COLOR).grid(row=1, column=0, pady=5)
    entry_a2 = tk.Entry(angle_frame, width=8, justify="center"); entry_a2.grid(row=1, column=1, padx=5)
    
    tk.Label(angle_frame, text="Z (mm):", font=font_label, bg=BG_COLOR).grid(row=2, column=0, pady=5)
    entry_az = tk.Entry(angle_frame, width=8, justify="center"); entry_az.grid(row=2, column=1, padx=5)

    def move_to_angles():
        try:
            j1, j2, z = float(entry_a1.get()), float(entry_a2.get()), float(entry_az.get())
            send_command(f"A,{j1},{j2},{z}")
        except ValueError:
            messagebox.showerror("Lỗi Nhập Liệu", "Vui lòng nhập CẢ 3 Ô (J1, J2, Z) bằng số hợp lệ!")

    tk.Button(angle_frame, text="ĐI TỚI GÓC NÀY", font=font_btn, bg="#27ae60", fg=BTN_TEXT, command=move_to_angles).grid(row=3, column=0, columnspan=2, pady=10, sticky="we")
    
    def homing_and_clear():
        global cmd_queue, is_busy
        cmd_queue.clear()
        lbl_queue.config(text="Lệnh chờ: 0")
        send_command("H", force=True) 

    tk.Button(left_frame, text="DỪNG LỆNH & VỀ GỐC", font=font_btn, bg="#c0392b", fg=BTN_TEXT, width=20, height=2, command=homing_and_clear).pack(pady=15)

    # ==========================================
    # CỘT 2: ĐỘNG HỌC NGƯỢC
    # ==========================================
    tk.Label(middle_frame, text="Động Học Ngược", font=font_subtitle, bg=BG_COLOR, fg="#8e44ad").pack(pady=(0, 20))
    
    ik_frame = tk.LabelFrame(middle_frame, text=" Di chuyển Tọa Độ XY ", font=font_label, bg=BG_COLOR, padx=15, pady=15)
    ik_frame.pack(fill=tk.X, pady=10)
    
    tk.Label(ik_frame, text="X (mm):", font=font_label, bg=BG_COLOR).grid(row=0, column=0, pady=10)
    entry_ik_x = tk.Entry(ik_frame, font=font_label, width=8, justify="center")
    entry_ik_x.grid(row=0, column=1, padx=10)
    
    tk.Label(ik_frame, text="Y (mm):", font=font_label, bg=BG_COLOR).grid(row=1, column=0, pady=10)
    entry_ik_y = tk.Entry(ik_frame, font=font_label, width=8, justify="center")
    entry_ik_y.grid(row=1, column=1, padx=10)

    def move_to_xyz():
        try:
            x, y = float(entry_ik_x.get()), float(entry_ik_y.get())
            send_command(f"P,{x},{y}")
        except ValueError:
            messagebox.showerror("Lỗi Nhập Liệu", "Vui lòng nhập đầy đủ 2 Ô (X, Y) bằng số hợp lệ!")
            
    tk.Button(ik_frame, text="TỚI TỌA ĐỘ XY", font=font_btn, bg=BTN_COLOR, fg=BTN_TEXT, height=2, command=move_to_xyz).grid(row=2, column=0, columnspan=2, pady=15, sticky="we")

    # ==========================================
    # CỘT 3: CÔNG CỤ VẼ & VIẾT CHỮ
    # ==========================================
    tk.Label(right_frame, text="Công Cụ Plotter", font=font_subtitle, bg=BG_COLOR, fg="#d35400").pack(pady=(0, 20))
    
    draw_box = tk.LabelFrame(right_frame, text=" Chọn hình cần vẽ ", font=font_label, bg=BG_COLOR, padx=15, pady=15)
    draw_box.pack(fill=tk.X)

    z_frame = tk.Frame(draw_box, bg=BG_COLOR)
    z_frame.pack(fill=tk.X, pady=(0, 10))
    
    tk.Label(z_frame, text="Z Nhấc an toàn:", bg=BG_COLOR).grid(row=0, column=0)
    entry_z_up = tk.Entry(z_frame, width=5, justify="center")
    entry_z_up.insert(0, "20.0")
    entry_z_up.grid(row=0, column=1, padx=5)
    
    tk.Label(z_frame, text=" Z Hạ mặt giấy:", bg=BG_COLOR).grid(row=0, column=2, padx=(10,0))
    entry_z_down = tk.Entry(z_frame, width=5, justify="center")
    entry_z_down.insert(0, "0.0")
    entry_z_down.grid(row=0, column=3, padx=5)

    draw_mode = tk.IntVar(value=3) 

    line_frame = tk.Frame(draw_box, bg=BG_COLOR)
    tk.Label(line_frame, text="X Bắt đầu:", bg=BG_COLOR).grid(row=0, column=0, pady=5)
    entry_x1 = tk.Entry(line_frame, width=7); entry_x1.grid(row=0, column=1)
    tk.Label(line_frame, text=" Y Bắt đầu:", bg=BG_COLOR).grid(row=0, column=2)
    entry_y1 = tk.Entry(line_frame, width=7); entry_y1.grid(row=0, column=3)
    tk.Label(line_frame, text="X Kết thúc:", bg=BG_COLOR).grid(row=1, column=0, pady=5)
    entry_x2 = tk.Entry(line_frame, width=7); entry_x2.grid(row=1, column=1)
    tk.Label(line_frame, text=" Y Kết thúc:", bg=BG_COLOR).grid(row=1, column=2)
    entry_y2 = tk.Entry(line_frame, width=7); entry_y2.grid(row=1, column=3)

    circle_frame = tk.Frame(draw_box, bg=BG_COLOR)
    tk.Label(circle_frame, text="Tâm X:", bg=BG_COLOR).grid(row=0, column=0, pady=5)
    entry_cx = tk.Entry(circle_frame, width=7); entry_cx.grid(row=0, column=1)
    tk.Label(circle_frame, text=" Tâm Y:", bg=BG_COLOR).grid(row=0, column=2)
    entry_cy = tk.Entry(circle_frame, width=7); entry_cy.grid(row=0, column=3)
    tk.Label(circle_frame, text="Bán kính R:", bg=BG_COLOR).grid(row=1, column=0, pady=5)
    entry_r = tk.Entry(circle_frame, width=7); entry_r.grid(row=1, column=1)

    text_frame = tk.Frame(draw_box, bg=BG_COLOR)
    tk.Label(text_frame, text="Nội dung:", bg=BG_COLOR).grid(row=0, column=0, pady=5, sticky="e")
    entry_text = tk.Entry(text_frame, width=18)
    entry_text.insert(0, "HELLO")
    entry_text.grid(row=0, column=1, columnspan=3, sticky="w")

    tk.Label(text_frame, text="X Bắt đầu:", bg=BG_COLOR).grid(row=1, column=0, pady=5, sticky="e")
    entry_tx = tk.Entry(text_frame, width=7); entry_tx.grid(row=1, column=1)
    tk.Label(text_frame, text=" Y Bắt đầu:", bg=BG_COLOR).grid(row=1, column=2, sticky="e")
    entry_ty = tk.Entry(text_frame, width=7); entry_ty.grid(row=1, column=3)

    tk.Label(text_frame, text="Cao chữ (mm):", bg=BG_COLOR).grid(row=2, column=0, pady=5, sticky="e")
    entry_th = tk.Entry(text_frame, width=7)
    entry_th.insert(0, "20")
    entry_th.grid(row=2, column=1)
    tk.Label(text_frame, text=" Cách chữ:", bg=BG_COLOR).grid(row=2, column=2, sticky="e")
    entry_tspace = tk.Entry(text_frame, width=7)
    entry_tspace.insert(0, "4")
    entry_tspace.grid(row=2, column=3)

    def toggle_draw_mode():
        line_frame.pack_forget()
        circle_frame.pack_forget()
        text_frame.pack_forget()
        if draw_mode.get() == 1:
            line_frame.pack(pady=10)
        elif draw_mode.get() == 2:
            circle_frame.pack(pady=10)
        else:
            text_frame.pack(pady=10)

    tk.Radiobutton(draw_box, text="Đường thẳng (Line)", variable=draw_mode, value=1, bg=BG_COLOR, font=font_label, command=toggle_draw_mode).pack(anchor="w")
    tk.Radiobutton(draw_box, text="Hình tròn (Circle)", variable=draw_mode, value=2, bg=BG_COLOR, font=font_label, command=toggle_draw_mode).pack(anchor="w")
    tk.Radiobutton(draw_box, text="Viết chữ (Text)", variable=draw_mode, value=3, bg=BG_COLOR, font=font_label, command=toggle_draw_mode).pack(anchor="w")
    toggle_draw_mode()

    def send_draw():
        try:
            z_up = float(entry_z_up.get())
            z_down = float(entry_z_down.get())
        except ValueError:
            messagebox.showerror("Lỗi", "Vui lòng nhập Z Nhấc và Z Hạ bằng số hợp lệ!")
            return

        if draw_mode.get() == 1:
            try:
                x1, y1 = float(entry_x1.get()), float(entry_y1.get())
                x2, y2 = float(entry_x2.get()), float(entry_y2.get())
                send_command(f"D,{x1:.1f},{y1:.1f},{x2:.1f},{y2:.1f},{z_up:.1f},{z_down:.1f}") 
            except ValueError:
                messagebox.showerror("Lỗi", "Vui lòng nhập đủ các thông số Line bằng số!")
        
        elif draw_mode.get() == 2:
            try:
                cx, cy, r = float(entry_cx.get()), float(entry_cy.get()), float(entry_r.get())
                send_command(f"C,{cx:.1f},{cy:.1f},{r:.1f},{z_up:.1f},{z_down:.1f}") 
            except ValueError:
                messagebox.showerror("Lỗi", "Vui lòng nhập đủ thông số Circle bằng số!")
        
        elif draw_mode.get() == 3:
            try:
                text_val = entry_text.get().upper()
                start_x = float(entry_tx.get())
                start_y = float(entry_ty.get())
                height = float(entry_th.get())
                spacing = float(entry_tspace.get())
                
                width = height * 0.6 
                current_x_offset = 0
                
                for char in text_val:
                    if char == ' ':
                        current_x_offset += width + spacing
                        continue
                        
                    if char in VECTOR_FONT:
                        lines = VECTOR_FONT[char]
                        for line in lines:
                            lx1 = start_x + current_x_offset + (line[0] * width)
                            ly1 = start_y + (line[1] * height)
                            lx2 = start_x + current_x_offset + (line[2] * width)
                            ly2 = start_y + (line[3] * height)
                            
                            cmd = f"D,{lx1:.1f},{ly1:.1f},{lx2:.1f},{ly2:.1f},{z_up:.1f},{z_down:.1f}"
                            send_command(cmd)
                            
                    current_x_offset += width + spacing

            except ValueError:
                messagebox.showerror("Lỗi", "Vui lòng nhập tọa độ X, Y Bắt đầu bằng số (VD: 150)!")

    tk.Button(draw_box, text="BẮT ĐẦU VẼ", font=font_btn, bg="#d35400", fg=BTN_TEXT, width=20, height=2, command=send_draw).pack(pady=15)

    root.after(50, read_serial)
    root.mainloop()

if __name__ == "__main__":
    create_ui_v2()