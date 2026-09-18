# FPGA-Accelerated-Edge-AI-Based-Multi-Task-Perception-System-For-Autonomus-System-Using-CNN-
The system integrates YOLOv8 models for vehicles, traffic signs, and lanes with FPGA hardware control. Real‑time detections trigger STOP/GO signals via UART to Spartan‑6 FPGA LEDs, achieving &lt;2 ms response. It enables cost‑effective edge AI for smart traffic safety.
Here VS code ;
from ultralytics import YOLO
import cv2
import numpy as np
import serial

# ─── UART Setup ───
ser = serial.Serial('COM4', 9600, timeout=1)
print("UART Connected!")

# ─── YOLO Model ───
model = YOLO("best.pt")

# ─── Lane Detection ───
def detect_lanes(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)
    height, width = frame.shape[:2]
    mask = np.zeros_like(edges)
    polygon = np.array([[
        (0, height), (width, height),
        (width//2+100, height//2),
        (width//2-100, height//2)
    ]], np.int32)
    cv2.fillPoly(mask, polygon, 255)
    masked = cv2.bitwise_and(edges, mask)
    lines = cv2.HoughLinesP(masked, 1, np.pi/180,
                             50, minLineLength=50, maxLineGap=150)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(frame, (x1,y1), (x2,y2), (0,255,0), 3)
    return frame

# ─── Main Loop ───
cap = cv2.VideoCapture("road.mp4")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Lane Detection
    frame = detect_lanes(frame)

    # Object Detection
    results = model(frame, conf=0.25)
    annotated_frame = results[0].plot()

    count = len(results[0].boxes)
    cv2.putText(annotated_frame, f"Objects: {count}",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                1, (0,255,0), 2)

    # ─── FPGA Data Transfer ───
    if count > 0:
        ser.write(b'1')   # Object detected → LED STOP ON
        print(f"Sent: 1 | Objects: {count}")
    else:
        ser.write(b'0')   # Clear → LED GO ON
        print("Sent: 0 | Road Clear")

    cv2.imshow("FPGA Edge AI", annotated_frame)

    if cv2.waitKey(1) & 0xFF == 27:
        break

# ─── Cleanup ───
cap.release()
ser.close()
cv2.destroyAllWindows()
print("System Stopped.")

Here VERILOG code
module fpga_control (
    input wire clk,
    input wire uart_rx,
    output reg led_stop,
    output reg led_go,
    output reg buzzer
);

parameter BAUD_DIV = 5208;

reg [12:0] baud_cnt = 0;
reg [3:0]  bit_cnt  = 0;
reg [7:0]  rx_shift = 0;
reg [7:0]  rx_data  = 0;
reg        rx_busy  = 0;
reg        rx_prev  = 1;

always @(posedge clk) begin
    rx_prev <= uart_rx;

    if (!rx_busy) begin
        if (rx_prev && !uart_rx) begin
            rx_busy  <= 1;
            baud_cnt <= BAUD_DIV / 2;
            bit_cnt  <= 0;
        end
    end else begin
        if (baud_cnt == 0) begin
            baud_cnt <= BAUD_DIV;
            if (bit_cnt < 8) begin
                rx_shift <= {uart_rx, rx_shift[7:1]};
                bit_cnt  <= bit_cnt + 1;
            end else begin
                rx_data <= rx_shift;
                rx_busy <= 0;
            end
        end else begin
            baud_cnt <= baud_cnt - 1;
        end
    end

    // LED Control
    if (rx_data == 8'h31) begin
        led_stop <= 1;
        led_go   <= 0;
        buzzer   <= 1;
    end else begin
        led_stop <= 0;
        led_go   <= 1;
        buzzer   <= 0;
    end
end

endmodule
