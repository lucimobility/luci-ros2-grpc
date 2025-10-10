#!/usr/bin/env python3

import tkinter as tk
from tkinter import scrolledtext, ttk
import subprocess
import signal
import os
import threading
import time
import math


class LuciControlUI:
    def __init__(self, root):
        self.root = root
        self.root.title("LUCI gRPC Interface Control")
        self.root.geometry("800x600")
        
        # Store PIDs
        self.grpc_pid = None
        self.joystick_pid = None
        
        # Movement tracking
        self.is_moving = False
        self.current_fb = 0  # Current forward/back value
        self.current_lr = 0  # Current left/right value
        
        # Joystick control variables
        self.joystick_active = False
        self.joystick_center_x = 0
        self.joystick_center_y = 0
        self.joystick_radius = 100
        self.joystick_handle_radius = 15
        self.joystick_handle_x = 0
        self.joystick_handle_y = 0
        
        # Keep track of active keys to prevent multiple instances
        self.active_keys = set()
        
        # For debouncing
        self.last_command_time = 0
        self.debounce_delay = 100  # milliseconds
        
        # Set up UI layout
        self.setup_ui()
        
        # Set up keyboard bindings
        self.setup_keyboard_bindings()
        
        # Set up cleanup on close
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Initial log message
        self.log("LUCI gRPC Interface Control UI started")
        self.log("Ready to connect to LUCI robot")
        self.log("Use the joystick or arrow keys to control the robot")
        
        # Start the movement update loop
        self.update_movement()

    def setup_ui(self):
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Left panel - Controls
        controls_frame = ttk.LabelFrame(main_frame, text="Controls", padding="10")
        controls_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))

        # gRPC Node Controls
        grpc_frame = ttk.LabelFrame(controls_frame, text="gRPC Node", padding="10")
        grpc_frame.pack(fill=tk.X, pady=(0, 10))

        # IP address entry
        ttk.Label(grpc_frame, text="IP Address:").pack(side=tk.LEFT, padx=(0, 2))
        self.grpc_ip_var = tk.StringVar(value="10.2.10.3")
        self.grpc_ip_entry = ttk.Entry(grpc_frame, textvariable=self.grpc_ip_var, width=15)
        self.grpc_ip_entry.pack(side=tk.LEFT, padx=2)

        ttk.Button(grpc_frame, text="Start Node", command=self.start_grpc_node).pack(side=tk.LEFT, padx=5)
        ttk.Button(grpc_frame, text="Stop Node", command=self.stop_grpc_node).pack(side=tk.LEFT, padx=5)
        
        # Movement Controls with Joystick
        movement_frame = ttk.LabelFrame(controls_frame, text="Movement Control", padding="10")
        movement_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Create joystick canvas
        joystick_frame = ttk.Frame(movement_frame)
        joystick_frame.pack(pady=10)
        
        # Canvas for the joystick
        self.joystick_canvas = tk.Canvas(joystick_frame, width=250, height=250, bg="#f0f0f0", highlightthickness=1, highlightbackground="#cccccc")
        self.joystick_canvas.pack(pady=10)
        
        # Set up the joystick
        self.setup_joystick()
        
        # Stop button below the joystick
        self.btn_stop = ttk.Button(movement_frame, text="■ Stop (Space)", width=15, command=self.stop_all_movement)
        self.btn_stop.pack(pady=5)
        
        # Keyboard controls info
        keyboard_info = ttk.Label(movement_frame, 
                                 text="Drag the joystick to control the robot.\nRelease to return to center.\nUse arrow keys as alternative.",
                                 justify=tk.CENTER)
        keyboard_info.pack(pady=5)
        
        # Movement speed control
        speed_frame = ttk.Frame(movement_frame)
        speed_frame.pack(fill=tk.X, pady=(10, 0))
        
        ttk.Label(speed_frame, text="Speed:").pack(side=tk.LEFT)
        self.speed_var = tk.IntVar(value=40)
        self.speed_scale = ttk.Scale(speed_frame, from_=10, to=100, orient=tk.HORIZONTAL, 
                                    variable=self.speed_var, length=200)
        self.speed_scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        self.speed_label = ttk.Label(speed_frame, text="40%")
        self.speed_label.pack(side=tk.LEFT, padx=5)
        
        # Update label when slider changes
        self.speed_var.trace_add("write", self.update_speed_label)
        
        # Service Controls
        service_frame = ttk.LabelFrame(controls_frame, text="Service Calls", padding="10")
        service_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Button(service_frame, text="Set Auto Remote Input", 
                  command=lambda: self.call_service("/luci/set_auto_remote_input")).pack(fill=tk.X, pady=2)
        ttk.Button(service_frame, text="Remove Auto Remote Input", 
                  command=lambda: self.call_service("/luci/remove_auto_remote_input")).pack(fill=tk.X, pady=2)
        
        # Information Controls
        info_frame = ttk.LabelFrame(controls_frame, text="Information", padding="10")
        info_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Button(info_frame, text="Show Topics", command=self.show_topics).pack(fill=tk.X, pady=2)
        ttk.Button(info_frame, text="Show Services", command=self.show_services).pack(fill=tk.X, pady=2)
        
        # Status indicator
        self.status_frame = ttk.Frame(controls_frame)
        self.status_frame.pack(fill=tk.X, pady=5)
        
        self.status_label = ttk.Label(self.status_frame, text="Status: Ready", font=("Arial", 10, "bold"))
        self.status_label.pack(side=tk.LEFT)
        
        self.status_indicator = tk.Canvas(self.status_frame, width=20, height=20, bg=self.root.cget("background"))
        self.status_indicator.pack(side=tk.LEFT, padx=5)
        self.status_circle = self.status_indicator.create_oval(5, 5, 15, 15, fill="gray")
        
        # Right panel - Log output
        log_frame = ttk.LabelFrame(main_frame, text="Log Output", padding="10")
        log_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(5, 0))
        
        # Scrolled text widget for log
        self.log_text = scrolledtext.ScrolledText(log_frame, wrap=tk.WORD, height=30)
        self.log_text.pack(fill=tk.BOTH, expand=True)
        self.log_text.config(state=tk.DISABLED)
        
        # Clear log button
        ttk.Button(log_frame, text="Clear Log", command=self.clear_log).pack(fill=tk.X, pady=(5, 0))

    def setup_joystick(self):
        """Set up the joystick UI elements"""
        # Get the canvas dimensions
        canvas_width = self.joystick_canvas.winfo_reqwidth()
        canvas_height = self.joystick_canvas.winfo_reqheight()
        
        # Calculate center point and radius
        self.joystick_center_x = canvas_width // 2
        self.joystick_center_y = canvas_height // 2
        self.joystick_radius = min(canvas_width, canvas_height) // 3
        
        # Draw outer circle (joystick boundary)
        self.joystick_boundary = self.joystick_canvas.create_oval(
            self.joystick_center_x - self.joystick_radius,
            self.joystick_center_y - self.joystick_radius,
            self.joystick_center_x + self.joystick_radius,
            self.joystick_center_y + self.joystick_radius,
            outline="#666666", width=2
        )
        
        # Draw crosshair lines
        self.joystick_canvas.create_line(
            self.joystick_center_x, 
            self.joystick_center_y - self.joystick_radius,
            self.joystick_center_x,
            self.joystick_center_y + self.joystick_radius,
            fill="#cccccc", dash=(4, 4)
        )
        self.joystick_canvas.create_line(
            self.joystick_center_x - self.joystick_radius,
            self.joystick_center_y,
            self.joystick_center_x + self.joystick_radius,
            self.joystick_center_y,
            fill="#cccccc", dash=(4, 4)
        )
        
        # Direction labels
        self.joystick_canvas.create_text(
            self.joystick_center_x,
            self.joystick_center_y - self.joystick_radius - 15,
            text="Forward", fill="#666666"
        )
        self.joystick_canvas.create_text(
            self.joystick_center_x,
            self.joystick_center_y + self.joystick_radius + 15,
            text="Backward", fill="#666666"
        )
        self.joystick_canvas.create_text(
            self.joystick_center_x - self.joystick_radius - 15,
            self.joystick_center_y,
            text="Left", fill="#666666", angle=90
        )
        self.joystick_canvas.create_text(
            self.joystick_center_x + self.joystick_radius + 15,
            self.joystick_center_y,
            text="Right", fill="#666666", angle=270
        )
        
        # Draw the handle (draggable part)
        self.joystick_handle_x = self.joystick_center_x
        self.joystick_handle_y = self.joystick_center_y
        self.joystick_handle = self.joystick_canvas.create_oval(
            self.joystick_handle_x - self.joystick_handle_radius,
            self.joystick_handle_y - self.joystick_handle_radius,
            self.joystick_handle_x + self.joystick_handle_radius,
            self.joystick_handle_y + self.joystick_handle_radius,
            fill="#3366cc", outline="#000000"
        )
        
        # Bind mouse events
        self.joystick_canvas.bind("<ButtonPress-1>", self.joystick_press)
        self.joystick_canvas.bind("<B1-Motion>", self.joystick_drag)
        self.joystick_canvas.bind("<ButtonRelease-1>", self.joystick_release)

    def joystick_press(self, event):
        """Handle joystick press event"""
        # Check if press is within the joystick area
        dx = event.x - self.joystick_center_x
        dy = event.y - self.joystick_center_y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance <= self.joystick_radius + self.joystick_handle_radius:
            self.joystick_active = True
            self.joystick_drag(event)  # Update immediately

    def joystick_drag(self, event):
        """Handle joystick drag event"""
        if not self.joystick_active:
            return
            
        # Calculate new position ensuring it's within the joystick boundary
        dx = event.x - self.joystick_center_x
        dy = event.y - self.joystick_center_y
        distance = math.sqrt(dx**2 + dy**2)
        
        # Limit to the boundary
        if distance > self.joystick_radius:
            dx = dx * (self.joystick_radius / distance)
            dy = dy * (self.joystick_radius / distance)
            
        # Update handle position
        self.joystick_handle_x = self.joystick_center_x + dx
        self.joystick_handle_y = self.joystick_center_y + dy
        
        # Move the joystick handle
        self.joystick_canvas.coords(
            self.joystick_handle,
            self.joystick_handle_x - self.joystick_handle_radius,
            self.joystick_handle_y - self.joystick_handle_radius,
            self.joystick_handle_x + self.joystick_handle_radius,
            self.joystick_handle_y + self.joystick_handle_radius
        )
        
        # Calculate joystick values (-100 to 100 range)
        # Invert Y axis because screen coordinates are opposite of joystick coordinates
        norm_x = dx / self.joystick_radius
        norm_y = -dy / self.joystick_radius  # Negative because up is negative Y in canvas
        
        # Convert to joystick values
        self.current_lr = int(norm_x * 100)
        self.current_fb = int(norm_y * 100)
        
        # Update movement flag
        if not self.is_moving:
            self.is_moving = True
            self.update_status()

    def joystick_release(self, event):
        """Handle joystick release event"""
        if self.joystick_active:
            self.joystick_active = False
            
            # Return joystick to center
            self.joystick_handle_x = self.joystick_center_x
            self.joystick_handle_y = self.joystick_center_y
            
            # Move the joystick handle back to center
            self.joystick_canvas.coords(
                self.joystick_handle,
                self.joystick_handle_x - self.joystick_handle_radius,
                self.joystick_handle_y - self.joystick_handle_radius,
                self.joystick_handle_x + self.joystick_handle_radius,
                self.joystick_handle_y + self.joystick_handle_radius
            )
            
            # Stop movement
            self.current_fb = 0
            self.current_lr = 0
            self.is_moving = False
            self.update_status()
            
            # Send stop command
            self.publish_movement(0, 0)

    def setup_keyboard_bindings(self):
        """Set up keyboard bindings for arrow keys"""
        # Arrow keys for movement
        self.root.bind('<KeyPress-Up>', lambda e: self.handle_key_press('Up', 100, 0))
        self.root.bind('<KeyRelease-Up>', lambda e: self.handle_key_release('Up', 100, 0))
        
        self.root.bind('<KeyPress-Down>', lambda e: self.handle_key_press('Down', -100, 0))
        self.root.bind('<KeyRelease-Down>', lambda e: self.handle_key_release('Down', -100, 0))
        
        self.root.bind('<KeyPress-Left>', lambda e: self.handle_key_press('Left', 0, -100))
        self.root.bind('<KeyRelease-Left>', lambda e: self.handle_key_release('Left', 0, -100))
        
        self.root.bind('<KeyPress-Right>', lambda e: self.handle_key_press('Right', 0, 100))
        self.root.bind('<KeyRelease-Right>', lambda e: self.handle_key_release('Right', 0, 100))
        
        # Space key for emergency stop
        self.root.bind('<space>', lambda e: self.stop_all_movement())
        
        # Focus on root to ensure keyboard events are caught
        self.root.focus_set()

    def handle_key_press(self, key, fb, lr):
        """Handle key press events to start movement"""
        if key not in self.active_keys:
            self.active_keys.add(key)
            self.handle_direction_press(fb, lr)
            
            # Update joystick position to match keyboard input
            if key == 'Up':
                self.move_joystick_ui(0, -self.joystick_radius)
            elif key == 'Down':
                self.move_joystick_ui(0, self.joystick_radius)
            elif key == 'Left':
                self.move_joystick_ui(-self.joystick_radius, 0)
            elif key == 'Right':
                self.move_joystick_ui(self.joystick_radius, 0)

    def handle_key_release(self, key, fb, lr):
        """Handle key release events to stop movement"""
        if key in self.active_keys:
            self.active_keys.remove(key)
            self.handle_direction_release(fb, lr)
            
            # If all keys released, reset joystick position
            if len(self.active_keys) == 0:
                self.move_joystick_ui(0, 0)  # Center the joystick

    def move_joystick_ui(self, dx, dy):
        """Move the joystick handle on the UI"""
        self.joystick_handle_x = self.joystick_center_x + dx
        self.joystick_handle_y = self.joystick_center_y + dy
        
        # Move the joystick handle
        self.joystick_canvas.coords(
            self.joystick_handle,
            self.joystick_handle_x - self.joystick_handle_radius,
            self.joystick_handle_y - self.joystick_handle_radius,
            self.joystick_handle_x + self.joystick_handle_radius,
            self.joystick_handle_y + self.joystick_handle_radius
        )

    def handle_direction_press(self, fb, lr):
        """Update movement direction when a key/button is pressed"""
        # Apply direction based on which button/key is pressed
        if fb != 0:  # Forward/backward movement
            self.current_fb = fb
        if lr != 0:  # Left/right movement
            self.current_lr = lr
            
        # Update movement state flag
        self.is_moving = True
        self.update_status()

    def handle_direction_release(self, fb, lr):
        """Handle when a direction button/key is released"""
        # Reset the appropriate direction
        if fb != 0 and self.current_fb == fb:  # If releasing forward/backward
            self.current_fb = 0
        if lr != 0 and self.current_lr == lr:  # If releasing left/right
            self.current_lr = 0
            
        # If no active keys left, make sure movement stops
        if len(self.active_keys) == 0:
            # Both directions are zero, we're stopped
            if self.current_fb == 0 and self.current_lr == 0:
                self.is_moving = False
                self.update_status()

    def update_movement(self):
        """Update the robot movement based on current direction values"""
        # Only send movement commands when the values change or we're moving
        if self.is_moving:
            speed_factor = self.speed_var.get() / 100.0  # Speed as percentage
            
            # Apply speed factor to current values
            scaled_fb = int(self.current_fb * speed_factor)
            scaled_lr = int(self.current_lr * speed_factor)
            
            # Only publish if we need to
            current_time = time.time() * 1000
            if current_time - self.last_command_time > self.debounce_delay:
                # Send movement command
                self.publish_movement(scaled_fb, scaled_lr)
                self.last_command_time = current_time
        
        # Schedule next update
        self.root.after(50, self.update_movement)

    def publish_movement(self, fb, lr):
        """Send movement command to the robot"""
        try:
            # Use shell=True for better performance
            cmd = f"ros2 topic pub --once /luci/remote_joystick luci_messages/msg/LuciJoystick \"{{forward_back: {fb}, left_right: {lr}, input_source: 1}}\""
            
            # Run in the background
            subprocess.Popen(
                cmd,
                shell=True,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
        except Exception as e:
            self.log(f"Error publishing movement: {e}")

    def update_speed_label(self, *args):
        speed = self.speed_var.get()
        self.speed_label.config(text=f"{speed}%")

    def log(self, message):
        """Add message to log with timestamp - non-blocking version"""
        timestamp = time.strftime("%H:%M:%S")
        log_message = f"[{timestamp}] {message}\n"
        
        # Use after() to update the log text to avoid blocking
        def update_log():
            self.log_text.config(state=tk.NORMAL)
            self.log_text.insert(tk.END, log_message)
            
            # Limit the log size
            if float(self.log_text.index('end-1c').split('.')[0]) > 500:
                self.log_text.delete(1.0, "100.0")
                
            self.log_text.see(tk.END)
            self.log_text.config(state=tk.DISABLED)
        
        self.root.after(1, update_log)

    def clear_log(self):
        """Clear the log text"""
        self.log_text.config(state=tk.NORMAL)
        self.log_text.delete(1.0, tk.END)
        self.log_text.config(state=tk.DISABLED)
        self.log("Log cleared")

    def start_grpc_node(self):
        if self.grpc_pid:
            self.log("gRPC node already running")
            return

        # Get IP from entry, default to 10.2.10.3 if blank
        ip = self.grpc_ip_var.get().strip() or "10.2.10.3"

        try:
            self.log(f"Starting LUCI gRPC node at IP: {ip} ...")
            process = subprocess.Popen(
                ["ros2", "run", "luci_grpc_interface", "grpc_interface_node", "-a", ip],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            self.grpc_pid = process.pid
            self.log(f"gRPC node started with PID: {self.grpc_pid}")
            self.update_status(is_connected=True)
        except Exception as e:
            self.log(f"Error starting gRPC node: {e}")
    
    def stop_grpc_node(self):
        if not self.grpc_pid:
            self.log("No gRPC node running")
            return
            
        try:
            self.log(f"Stopping gRPC node (PID: {self.grpc_pid})...")
            os.kill(self.grpc_pid, signal.SIGTERM)
            self.grpc_pid = None
            self.log("gRPC node stopped")
            self.update_status(is_connected=False)
        except Exception as e:
            self.log(f"Error stopping gRPC node: {e}")

    def stop_all_movement(self):
        """Emergency stop - stop all movement"""
        # Reset direction values
        self.current_fb = 0
        self.current_lr = 0
        self.is_moving = False
        
        # Clear active keys
        self.active_keys.clear()
        
        # Reset joystick position
        self.move_joystick_ui(0, 0)
        
        # Send stop command
        self.publish_movement(0, 0)
        self.log("Movement stopped")
        
        self.update_status()

    def call_service(self, service_name):
        """Call a ROS2 service"""
        self.log(f"Calling service: {service_name}")
        
        def service_thread():
            try:
                result = subprocess.run(
                    ["ros2", "service", "call", service_name, "std_srvs/srv/Empty"],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True
                )
                if result.returncode == 0:
                    self.log(f"Service call to {service_name} succeeded")
                else:
                    self.log(f"Service call failed: {result.stderr}")
            except Exception as e:
                self.log(f"Error calling service: {e}")
                
        # Run service call in a separate thread to avoid blocking UI
        threading.Thread(target=service_thread).start()

    def show_topics(self):
        """Show LUCI topics"""
        self.log("Retrieving LUCI topics...")
        
        def topic_thread():
            try:
                result = subprocess.run(
                    ["ros2", "topic", "list"],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True
                )
                if result.returncode == 0:
                    topics = [topic for topic in result.stdout.split('\n') if 'luci' in topic]
                    self.log("LUCI Topics:")
                    for topic in topics:
                        self.log(f"  • {topic}")
                else:
                    self.log(f"Error retrieving topics: {result.stderr}")
            except Exception as e:
                self.log(f"Error: {e}")
                
        threading.Thread(target=topic_thread).start()

    def show_services(self):
        """Show LUCI services"""
        self.log("Retrieving LUCI services...")
        
        def service_thread():
            try:
                result = subprocess.run(
                    ["ros2", "service", "list"],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True
                )
                if result.returncode == 0:
                    services = [service for service in result.stdout.split('\n') if 'luci' in service]
                    self.log("LUCI Services:")
                    for service in services:
                        self.log(f"  • {service}")
                else:
                    self.log(f"Error retrieving services: {result.stderr}")
            except Exception as e:
                self.log(f"Error: {e}")
                
        threading.Thread(target=service_thread).start()
        
    def update_status(self, is_connected=None):
        """Update status indicator"""
        if is_connected is not None:
            self.is_connected = is_connected
            
        # Update status based on connection and movement
        if not hasattr(self, 'is_connected'):
            self.is_connected = False
            
        if self.is_connected and self.is_moving:
            status_text = "Status: Connected & Moving"
            color = "green"
        elif self.is_connected:
            status_text = "Status: Connected"
            color = "blue"
        else:
            status_text = "Status: Disconnected"
            color = "gray"
            
        self.status_label.config(text=status_text)
        self.status_indicator.itemconfig(self.status_circle, fill=color)

    def on_closing(self):
        """Clean up before closing"""
        self.stop_all_movement()
        self.stop_grpc_node()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = LuciControlUI(root)
    root.mainloop()