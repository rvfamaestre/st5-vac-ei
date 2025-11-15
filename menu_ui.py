import ctypes
import io
import queue
import sys
import threading
import time
import traceback
import tkinter as tk
from tkinter import ttk

_PRE_IMPORT_STDOUT = sys.stdout
_PRE_IMPORT_STDERR = sys.stderr
_PRE_IMPORT_BUFFER = io.StringIO()
sys.stdout = _PRE_IMPORT_BUFFER
sys.stderr = _PRE_IMPORT_BUFFER
try:
    import dialogue
finally:
    sys.stdout = _PRE_IMPORT_STDOUT
    sys.stderr = _PRE_IMPORT_STDERR

PRE_IMPORT_LOG = _PRE_IMPORT_BUFFER.getvalue()


BG_MAIN = "#121212"
BG_PANEL = "#1b1b1b"
FG_TEXT = "#f5f5f5"
ACCENT = "#3a7afe"
ACCENT_SOFT = "#2c4faa"
ERROR = "#ff6b6b"
SUCCESS = "#4cd964"

# Variable globale pour permettre à dialogue.py de mettre à jour le robot
_global_grid_ui = None


class GuiConsole(io.TextIOBase):

    def __init__(self, text_widget: tk.Text):
        super().__init__()
        self.text = text_widget
        self.queue = queue.Queue()
        self.text.after(100, self._flush_queue)

    def write(self, message):
        if not message:
            return
        self.queue.put(message)

    def flush(self):
        pass

    def _flush_queue(self):
        try:
            while True:
                message = self.queue.get_nowait()
                self.text.configure(state="normal")
                self.text.insert("end", message)
                self.text.see("end")
                self.text.configure(state="disabled")
        except queue.Empty:
            pass
        finally:
            self.text.after(100, self._flush_queue)


class GridUI:

    NODE_RADIUS = 10
    EDGE_WIDTH = 2
    EDGE_WIDTH_BLOCKED = 5

    def __init__(self, canvas: tk.Canvas, selection_var: tk.StringVar):
        self.canvas = canvas
        self.selection_var = selection_var
        self.grid_size = dialogue.GRID_SIZE
        self.margin = 50
        self.size = int(canvas["width"])
        self.node_radius = self.NODE_RADIUS
        self.nodes = {}
        self.edges = {}
        self.start_node = None
        self.goal_node = None
        self.blocked_edges = set()
        self.path_edges = set()
        self.robot_marker = None  # Marqueur du robot
        self.robot_position = None  # Position actuelle du robot (x, y)
        self.robot_heading = None  # Orientation actuelle ('N', 'E', 'S', 'W')
        self._draw_grid()

    def _draw_grid(self):
        spacing = (self.size - 2 * self.margin) / (self.grid_size - 1)
        
        # Build edges (horizontal and vertical) FIRST so they are below nodes
        for y in range(self.grid_size):
            for x in range(self.grid_size):
                if x < self.grid_size - 1:
                    self._create_edge((x, y), (x + 1, y))
                if y < self.grid_size - 1:
                    self._create_edge((x, y), (x, y + 1))
        
        # Create nodes AFTER edges so they are on top and receive clicks first
        for y in range(self.grid_size):
            for x in range(self.grid_size):
                cx = self.margin + x * spacing
                cy = self.margin + (self.grid_size - 1 - y) * spacing
                node_tag = f"node_{x}_{y}"
                oval = self.canvas.create_oval(
                    cx - self.node_radius,
                    cy - self.node_radius,
                    cx + self.node_radius,
                    cy + self.node_radius,
                    fill=BG_MAIN,
                    outline=FG_TEXT,
                    width=2,
                    tags=("node", node_tag),
                )
                self.nodes[(x, y)] = oval
                self.canvas.tag_bind(
                    node_tag, "<Button-1>", lambda _evt, pos=(x, y): self._on_node_click(pos)
                )

    def _create_edge(self, a, b):
        ax, ay = self._node_center(a)
        bx, by = self._node_center(b)
        edge = dialogue.norm_edge(a, b)
        tag = f"edge_{edge[0][0]}_{edge[0][1]}_{edge[1][0]}_{edge[1][1]}"
        line = self.canvas.create_line(
            ax,
            ay,
            bx,
            by,
            fill=FG_TEXT,
            width=self.EDGE_WIDTH,
            tags=("edge", tag),
        )
        self.edges[edge] = line
        self.canvas.tag_bind(
            tag, "<Button-1>", lambda _evt, e=edge: self.toggle_edge(e)
        )

    def _node_center(self, pos):
        spacing = (self.size - 2 * self.margin) / (self.grid_size - 1)
        x, y = pos
        cx = self.margin + x * spacing
        cy = self.margin + (self.grid_size - 1 - y) * spacing
        return cx, cy

    def _on_node_click(self, pos):
        mode = self.selection_var.get()
        if mode == "start":
            if self.start_node == pos:
                # Si on clique sur le même nœud, on le désélectionne
                self.start_node = None
            else:
                # Nouveau point de départ sélectionné
                self.start_node = pos
                # Basculer automatiquement en mode "goal" pour plus de fluidité
                self.selection_var.set("goal")
        else:
            if self.goal_node == pos:
                # Si on clique sur le même nœud, on le désélectionne
                self.goal_node = None
            else:
                # Nouveau point d'arrivée sélectionné
                self.goal_node = pos
        self._refresh_nodes()

    def _refresh_nodes(self):
        for (x, y), item in self.nodes.items():
            fill = BG_MAIN
            outline = FG_TEXT
            if self.start_node == (x, y):
                fill = ACCENT
            if self.goal_node == (x, y):
                fill = ERROR if self.goal_node == self.start_node else SUCCESS
            self.canvas.itemconfigure(item, fill=fill, outline=FG_TEXT)

    def toggle_edge(self, edge):
        normalized = dialogue.norm_edge(*edge)
        if normalized in self.blocked_edges:
            self.blocked_edges.remove(normalized)
        else:
            self.blocked_edges.add(normalized)
        self._update_edge_visual(normalized)

    def _update_edge_visual(self, edge):
        item = self.edges.get(edge)
        if not item:
            return
        if edge in self.blocked_edges:
            color = ERROR
            width = self.EDGE_WIDTH_BLOCKED
        elif edge in self.path_edges:
            color = "#3dff6f"
            width = self.EDGE_WIDTH + 1
        else:
            color = FG_TEXT
            width = self.EDGE_WIDTH
        self.canvas.itemconfigure(item, fill=color, width=width)

    def _refresh_edges(self):
        for edge in self.edges.keys():
            self._update_edge_visual(edge)

    def mark_obstacle_async(self, edge):
        normalized = dialogue.norm_edge(*edge)
        self.canvas.after(0, lambda: self._mark_obstacle(normalized))

    def _mark_obstacle(self, edge):
        self.blocked_edges.add(edge)
        self._update_edge_visual(edge)

    def show_path_async(self, nodes):
        payload = None
        if nodes:
            payload = [tuple(n) for n in nodes]
        self.canvas.after(0, lambda: self._set_path_nodes(payload))

    def _set_path_nodes(self, nodes):
        if not nodes or len(nodes) < 2:
            self.path_edges.clear()
            self._refresh_edges()
            return
        edges = set()
        for a, b in zip(nodes, nodes[1:]):
            if a == b:
                continue
            edges.add(dialogue.norm_edge(tuple(a), tuple(b)))
        self.path_edges = edges
        self._refresh_edges()

    def reset(self):
        self.start_node = None
        self.goal_node = None
        self.path_edges.clear()
        self.blocked_edges.clear()
        self._refresh_nodes()
        self._refresh_edges()
        self.hide_robot()

    def get_blocked_edges(self):
        return list(self.blocked_edges)

    def update_robot_position(self, position, heading):
        self.robot_position = position
        self.robot_heading = heading
        self._draw_robot()

    def update_robot_async(self, position, heading):
        self.canvas.after(0, lambda: self.update_robot_position(position, heading))

    def _draw_robot(self):
        # Supprimer l'ancien marqueur
        if self.robot_marker:
            self.canvas.delete(self.robot_marker)
            self.robot_marker = None

        if not self.robot_position or not self.robot_heading:
            return

        # Calculer le centre du nœud
        cx, cy = self._node_center(self.robot_position)
        
        # Taille du triangle
        size = 15
        
        # Points du triangle selon l'orientation
        # N = pointe vers le haut, E = droite, S = bas, W = gauche
        if self.robot_heading == 'N':
            points = [cx, cy - size, cx - size/2, cy + size/2, cx + size/2, cy + size/2]
        elif self.robot_heading == 'E':
            points = [cx + size, cy, cx - size/2, cy - size/2, cx - size/2, cy + size/2]
        elif self.robot_heading == 'S':
            points = [cx, cy + size, cx - size/2, cy - size/2, cx + size/2, cy - size/2]
        else:  # 'W'
            points = [cx - size, cy, cx + size/2, cy - size/2, cx + size/2, cy + size/2]
        
        # Dessiner le triangle (robot)
        self.robot_marker = self.canvas.create_polygon(
            points,
            fill="#FFA500",  # Orange pour le robot
            outline="#FF8C00",
            width=2,
            tags="robot"
        )
        # Mettre le robot au premier plan
        self.canvas.tag_raise("robot")

    def hide_robot(self):
        if self.robot_marker:
            self.canvas.delete(self.robot_marker)
            self.robot_marker = None
        self.robot_position = None
        self.robot_heading = None


class Application:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("VAC Robot Control - Interface graphique")
        self.root.configure(bg=BG_MAIN)
        self.root.geometry("1180x780")

        self.arduino = None
        self.current_thread = None
        self.current_task_name = None
        self.stdout_backup = sys.stdout
        self.stderr_backup = sys.stderr

        self.status_var = tk.StringVar(value="Initialisation...")
        self.task_var = tk.StringVar(value="Aucune tache en cours")

        self.selection_mode = tk.StringVar(value="start")
        self.console_widget = None
        self.grid_ui = None

        self._build_layout()
        self._redirect_streams()

        if PRE_IMPORT_LOG.strip():
            for line in PRE_IMPORT_LOG.strip().splitlines():
                print(line)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(200, self._connect_arduino_async)

    def _build_layout(self):
        top_bar = tk.Frame(self.root, bg=BG_PANEL, padx=10, pady=10)
        top_bar.pack(side="top", fill="x")

        status_label = tk.Label(
            top_bar,
            textvariable=self.status_var,
            fg=FG_TEXT,
            bg=BG_PANEL,
            font=("Segoe UI", 11, "bold"),
        )
        status_label.pack(side="left")

        task_label = tk.Label(
            top_bar,
            textvariable=self.task_var,
            fg="#dddddd",
            bg=BG_PANEL,
            font=("Segoe UI", 10),
        )
        task_label.pack(side="right")

        main_frame = tk.Frame(self.root, bg=BG_MAIN)
        main_frame.pack(side="top", fill="both", expand=True)

        sidebar = tk.Frame(main_frame, bg=BG_MAIN, padx=10, pady=10)
        sidebar.pack(side="left", fill="y")

        self.content = tk.Frame(main_frame, bg=BG_MAIN, padx=10, pady=10)
        self.content.pack(side="left", fill="both", expand=True)
        self.content.grid_rowconfigure(0, weight=1)
        self.content.grid_columnconfigure(0, weight=1)

        buttons = [
            ("Accueil", "home"),
            ("Dialogue direct", "direct"),
            ("Suivi de ligne", "line"),
            ("Trajet predefini", "path"),
            ("Navigation A->B", "grid"),
        ]
        for text, target in buttons:
            btn = tk.Button(
                sidebar,
                text=text,
                command=lambda t=target: self.show_frame(t),
                width=18,
                pady=10,
                bg=ACCENT_SOFT,
                fg=FG_TEXT,
                relief="flat",
                activebackground=ACCENT,
                activeforeground=FG_TEXT,
                font=("Segoe UI", 10, "bold"),
            )
            btn.pack(fill="x", pady=6)

        refresh_btn = tk.Button(
            sidebar,
            text="Reconnecter Arduino",
            command=self._connect_arduino_async,
            bg=BG_PANEL,
            fg=FG_TEXT,
            relief="groove",
            activebackground=ACCENT,
            activeforeground=FG_TEXT,
        )
        refresh_btn.pack(fill="x", pady=(30, 0))

        stop_btn = tk.Button(
            sidebar,
            text="Arreter la tache",
            command=self.stop_task,
            bg=ERROR,
            fg=BG_MAIN,
            relief="flat",
            activebackground="#ff8585",
            activeforeground=BG_MAIN,
        )
        stop_btn.pack(fill="x", pady=(10, 0))

        # Content frames
        self.frames = {}
        for name in ("home", "direct", "line", "path", "grid"):
            frame = tk.Frame(self.content, bg=BG_MAIN)
            frame.grid(row=0, column=0, sticky="nsew")
            self.frames[name] = frame

        self._build_home()
        self._build_direct()
        self._build_line_follow()
        self._build_path_follow()
        self._build_grid_mode()

        log_frame = tk.Frame(self.root, bg=BG_PANEL, padx=10, pady=10)
        log_frame.pack(side="bottom", fill="both")

        tk.Label(
            log_frame,
            text="Journal d'execution",
            fg=FG_TEXT,
            bg=BG_PANEL,
            font=("Segoe UI", 10, "bold"),
        ).pack(anchor="w")

        text_frame = tk.Frame(log_frame, bg=BG_PANEL)
        text_frame.pack(fill="both", expand=True)

        log_text = tk.Text(
            text_frame,
            height=12,
            bg="#000000",
            fg=FG_TEXT,
            insertbackground=FG_TEXT,
            wrap="word",
            state="disabled",
            relief="flat",
        )
        log_text.pack(side="left", fill="both", expand=True)

        scrollbar = tk.Scrollbar(text_frame, command=log_text.yview)
        scrollbar.pack(side="right", fill="y")
        log_text.configure(yscrollcommand=scrollbar.set)

        clear_btn = tk.Button(
            log_frame,
            text="Effacer le journal",
            command=lambda: self._clear_log(log_text),
            bg=BG_PANEL,
            fg=FG_TEXT,
            relief="flat",
        )
        clear_btn.pack(anchor="e", pady=(6, 0))

        self.console_widget = log_text
        self.show_frame("home")

    def _build_home(self):
        frame = self.frames["home"]
        tk.Label(
            frame,
            text="Bienvenue dans l'interface graphique du robot VAC",
            bg=BG_MAIN,
            fg=FG_TEXT,
            font=("Segoe UI", 16, "bold"),
        ).pack(anchor="w", pady=(0, 20))

        instructions = [
            "- Choisissez un mode dans le menu de gauche.",
            "- Toutes les informations qui etaient imprimees en console arrivent ici dans le journal.",
            "- Utilisez le bouton \"Arreter la tache\" pour interrompre un mode en cours.",
            "- Pour la navigation A->B : selectionnez START et GOAL sur le reseau de noeuds, placez les obstacles en cliquant sur les aretes.",
        ]

        for text in instructions:
            tk.Label(frame, text=text, bg=BG_MAIN, fg=FG_TEXT, font=("Segoe UI", 12)).pack(
                anchor="w", pady=4
            )

    def _build_direct(self):
        frame = self.frames["direct"]
        tk.Label(
            frame,
            text="Dialogue direct avec l'Arduino",
            bg=BG_MAIN,
            fg=FG_TEXT,
            font=("Segoe UI", 14, "bold"),
        ).pack(anchor="w", pady=(0, 12))

        entry_frame = tk.Frame(frame, bg=BG_MAIN)
        entry_frame.pack(anchor="w", pady=10)

        tk.Label(entry_frame, text="Commande :", bg=BG_MAIN, fg=FG_TEXT).pack(
            side="left", padx=(0, 8)
        )
        self.direct_entry = tk.Entry(
            entry_frame, width=20, bg=BG_PANEL, fg=FG_TEXT, insertbackground=FG_TEXT
        )
        self.direct_entry.pack(side="left")

        send_btn = tk.Button(
            entry_frame,
            text="Envoyer",
            command=self._send_direct_entry,
            bg=ACCENT,
            fg=FG_TEXT,
            relief="flat",
        )
        send_btn.pack(side="left", padx=8)

        quick_frame = tk.Frame(frame, bg=BG_MAIN)
        quick_frame.pack(anchor="w", pady=12)

        quick_actions = [
            ("Demi-tour", "u"),
            ("Gauche", "l"),
            ("Droite", "r"),
            ("Avant 5s", "f"),
            ("Arriere 5s", "b"),
            ("Stop moteurs", "stop"),
        ]

        for label, command in quick_actions:
            tk.Button(
                quick_frame,
                text=label,
                command=lambda cmd=command: self.send_direct_command(cmd),
                bg=ACCENT_SOFT,
                fg=FG_TEXT,
                relief="flat",
                padx=10,
                pady=6,
            ).pack(side="left", padx=5)

        info = (
            "Les commandes textuelles sont envoyees telles quelles. "
            "Les boutons reproduisent les raccourcis du menu terminal."
        )
        tk.Label(frame, text=info, bg=BG_MAIN, fg="#bbbbbb", wraplength=600).pack(
            anchor="w", pady=10
        )

    def _build_line_follow(self):
        frame = self.frames["line"]
        tk.Label(
            frame,
            text="Mode suivi de ligne autonome",
            bg=BG_MAIN,
            fg=FG_TEXT,
            font=("Segoe UI", 14, "bold"),
        ).pack(anchor="w", pady=(0, 12))

        form = tk.Frame(frame, bg=BG_MAIN)
        form.pack(anchor="w", pady=8)

        tk.Label(form, text="Duree (0 = infini) :", bg=BG_MAIN, fg=FG_TEXT).grid(
            row=0, column=0, sticky="w"
        )
        self.line_duration = tk.Entry(form, width=8, bg=BG_PANEL, fg=FG_TEXT, insertbackground=FG_TEXT)
        self.line_duration.insert(0, "60")
        self.line_duration.grid(row=0, column=1, padx=(6, 12))

        self.line_feedback = tk.BooleanVar(value=True)
        tk.Checkbutton(
            form,
            text="Feedback",
            variable=self.line_feedback,
            bg=BG_MAIN,
            fg=FG_TEXT,
            selectcolor=BG_PANEL,
        ).grid(row=0, column=2, padx=4)

        self.line_intersections = tk.BooleanVar(value=True)
        tk.Checkbutton(
            form,
            text="Decisions intersection",
            variable=self.line_intersections,
            bg=BG_MAIN,
            fg=FG_TEXT,
            selectcolor=BG_PANEL,
        ).grid(row=0, column=3, padx=4)

        self.line_obstacles = tk.BooleanVar(value=True)
        tk.Checkbutton(
            form,
            text="Arret obstacle",
            variable=self.line_obstacles,
            bg=BG_MAIN,
            fg=FG_TEXT,
            selectcolor=BG_PANEL,
        ).grid(row=0, column=4, padx=4)

        tk.Button(
            frame,
            text="Demarrer le suivi",
            command=self.start_line_following,
            bg=ACCENT,
            fg=FG_TEXT,
            relief="flat",
            padx=16,
            pady=8,
        ).pack(anchor="w", pady=(12, 0))

    def _build_path_follow(self):
        frame = self.frames["path"]
        tk.Label(
            frame,
            text="Mode trajet predefini",
            bg=BG_MAIN,
            fg=FG_TEXT,
            font=("Segoe UI", 14, "bold"),
        ).pack(anchor="w", pady=(0, 12))

        tk.Label(
            frame,
            text="Instructions (L=Gauche, R=Droite, S=Tout droit, B=Demi-tour) :",
            bg=BG_MAIN,
            fg=FG_TEXT,
        ).pack(anchor="w")

        self.path_entry = tk.Entry(
            frame, width=30, bg=BG_PANEL, fg=FG_TEXT, insertbackground=FG_TEXT, font=("Consolas", 12)
        )
        self.path_entry.pack(anchor="w", pady=8)

        tk.Button(
            frame,
            text="Executer le trajet",
            command=self.start_path_following,
            bg=ACCENT,
            fg=FG_TEXT,
            relief="flat",
            padx=16,
            pady=8,
        ).pack(anchor="w", pady=(4, 0))

    def _build_grid_mode(self):
        frame = self.frames["grid"]
        tk.Label(
            frame,
            text="Navigation de A a B",
            bg=BG_MAIN,
            fg=FG_TEXT,
            font=("Segoe UI", 14, "bold"),
        ).pack(anchor="w", pady=(0, 12))

        control_bar = tk.Frame(frame, bg=BG_MAIN)
        control_bar.pack(anchor="w", pady=8)

        tk.Label(control_bar, text="Selection :", bg=BG_MAIN, fg=FG_TEXT).pack(
            side="left", padx=(0, 6)
        )
        tk.Radiobutton(
            control_bar,
            text="Start",
            variable=self.selection_mode,
            value="start",
            bg=BG_MAIN,
            fg=FG_TEXT,
            selectcolor=BG_PANEL,
        ).pack(side="left")
        tk.Radiobutton(
            control_bar,
            text="Goal",
            variable=self.selection_mode,
            value="goal",
            bg=BG_MAIN,
            fg=FG_TEXT,
            selectcolor=BG_PANEL,
        ).pack(side="left", padx=(0, 10))

        tk.Label(control_bar, text="Orientation initiale :", bg=BG_MAIN, fg=FG_TEXT).pack(
            side="left", padx=(10, 6)
        )
        self.heading_var = tk.StringVar(value="N")
        heading_menu = ttk.Combobox(
            control_bar,
            textvariable=self.heading_var,
            values=["N", "E", "S", "W"],
            width=4,
            state="readonly",
        )
        heading_menu.pack(side="left")

        tk.Button(
            control_bar,
            text="Reinitialiser",
            command=self.reset_grid,
            bg=BG_PANEL,
            fg=FG_TEXT,
            relief="flat",
            padx=10,
        ).pack(side="left", padx=(10, 0))

        canvas_frame = tk.Frame(frame, bg=BG_MAIN)
        canvas_frame.pack(fill="both", expand=True, pady=10)

        canvas = tk.Canvas(
            canvas_frame,
            width=480,
            height=480,
            bg="black",
            highlightthickness=0,
        )
        canvas.pack()

        self.grid_ui = GridUI(canvas, self.selection_mode)
        
        # Exposer grid_ui globalement pour que dialogue.py puisse l'utiliser
        global _global_grid_ui
        _global_grid_ui = self.grid_ui

        action_bar = tk.Frame(frame, bg=BG_MAIN)
        action_bar.pack(anchor="w", pady=10)

        tk.Button(
            action_bar,
            text="Planifier et executer",
            command=self.start_ab_mode,
            bg=ACCENT,
            fg=FG_TEXT,
            relief="flat",
            padx=16,
            pady=8,
        ).pack(side="left")

    def show_frame(self, name):
        frame = self.frames.get(name)
        if frame:
            frame.tkraise()

    def _clear_log(self, widget):
        widget.configure(state="normal")
        widget.delete("1.0", "end")
        widget.configure(state="disabled")

    def _redirect_streams(self):
        console = GuiConsole(self.console_widget)
        sys.stdout = console
        sys.stderr = console

    def _connect_arduino_async(self):
        def worker():
            try:
                self._set_status("Connexion a l'Arduino...")
                self.arduino = dialogue.initialize_arduino()
                self._set_status("Arduino connecte")
            except Exception as exc:
                print(f"X Echec de connexion Arduino: {exc}")
                traceback.print_exc()
                self.arduino = None
                self._set_status("Arduino non connecte")

        threading.Thread(target=worker, daemon=True).start()

    def _set_status(self, text):
        self.root.after(0, lambda: self.status_var.set(text))

    def _set_task(self, text):
        self.root.after(0, lambda: self.task_var.set(text))

    def ensure_connection(self):
        if self.arduino and getattr(self.arduino, "is_open", False):
            return self.arduino
        try:
            self.arduino = dialogue.initialize_arduino()
            self._set_status("Arduino connecte")
            return self.arduino
        except Exception as exc:
            print(f"X Impossible de se connecter a l'Arduino: {exc}")
            traceback.print_exc()
            self._set_status("Arduino non connecte")
            return None

    def _send_direct_entry(self):
        command = self.direct_entry.get().strip()
        if not command:
            return
        self.send_direct_command(command)
        self.direct_entry.delete(0, "end")

    def send_direct_command(self, command):
        arduino = self.ensure_connection()
        if arduino is None:
            return
        command = command.strip()
        if not command:
            return

        try:
            if command == "stop":
                dialogue.send_motor_command(arduino, 0, 0)
                print("-> moteurs arretes")
                return
            if command.lower() == "u":
                dialogue.make_uturn(arduino)
                return
            if command.lower() == "l":
                dialogue.make_left_turn(arduino)
                return
            if command.lower() == "r":
                dialogue.make_right_turn(arduino)
                return
            if command.lower() == "f":
                dialogue.send_motor_command(arduino, 255, 255)
                time.sleep(5)
                dialogue.send_motor_command(arduino, 0, 0)
                print("-> avance de 5 secondes effectuee")
                return
            if command.lower() == "b":
                dialogue.send_motor_command(arduino, -255, -255)
                time.sleep(5)
                dialogue.send_motor_command(arduino, 0, 0)
                print("-> marche arriere de 5 secondes effectuee")
                return

            arduino.write(command.encode("utf-8"))
            time.sleep(0.02)
            self._read_serial_responses(arduino)
        except Exception as exc:
            print(f"X Erreur en envoyant la commande: {exc}")
            traceback.print_exc()

    def _read_serial_responses(self, arduino):
        rep = arduino.readline()
        attempts = 0
        while rep == b"" and attempts < 20:
            rep = arduino.readline()
            attempts += 1
        while rep:
            try:
                print(rep.decode(errors="ignore").strip())
            except Exception:
                print(rep)
            if arduino.inWaiting() <= 0:
                break
            rep = arduino.readline()

    def start_line_following(self):
        arduino = self.ensure_connection()
        if arduino is None:
            return
        try:
            duration = int(self.line_duration.get())
        except ValueError:
            print("X Duree invalide. Utilisation de 60 secondes.")
            duration = 60
        kwargs = dict(
            duration=duration,
            feedback=self.line_feedback.get(),
            enable_intersection_decision=self.line_intersections.get(),
            enable_obstacle_stop=self.line_obstacles.get(),
        )
        self._launch_task(
            "Suivi de ligne",
            dialogue.autonomous_line_following,
            arduino,
            **kwargs,
        )

    def start_path_following(self):
        arduino = self.ensure_connection()
        if arduino is None:
            return
        path = self.path_entry.get().strip().upper()
        if not path:
            print("X Merci de saisir une sequence d'instructions (L, R, S, B).")
            return
        self._launch_task(
            "Trajet predefini",
            dialogue.path_following_mode,
            arduino,
            path,
            True,
        )

    def start_ab_mode(self):
        arduino = self.ensure_connection()
        if arduino is None:
            return
        start = self.grid_ui.start_node
        goal = self.grid_ui.goal_node
        heading = self.heading_var.get()

        if start is None or goal is None:
            print("X Selectionnez un noeud de depart et un noeud d'arrivee.")
            return
        if heading not in {"N", "E", "S", "W"}:
            print("X Orientation initiale invalide.")
            return

        blocked = self.grid_ui.get_blocked_edges()

        self.grid_ui.show_path_async(None)
        self._launch_task(
            "Navigation A->B",
            dialogue.plan_and_go,
            arduino,
            start,
            goal,
            heading,
            blocked,
            obstacle_callback=lambda edge: self.grid_ui.mark_obstacle_async(edge),
            path_callback=lambda nodes: self.grid_ui.show_path_async(nodes),
        )

    def reset_grid(self):
        if self.grid_ui:
            self.grid_ui.reset()
        # Remettre le mode de sélection sur "start" pour recommencer
        self.selection_mode.set("start")

    def _launch_task(self, name, target, *args, **kwargs):
        if self.current_thread and self.current_thread.is_alive():
            print("X Une tache est deja en cours. Arretez-la avant d'en lancer une nouvelle.")
            return

        def run():
            try:
                target(*args, **kwargs)
            except KeyboardInterrupt:
                print("Tache interrompue par l'utilisateur.")
            except Exception as exc:
                print(f"X Erreur durant {name.lower()}: {exc}")
                traceback.print_exc()
            finally:
                self.current_thread = None
                self.current_task_name = None
                self._set_task("Aucune tache en cours")

        self.current_task_name = name
        self._set_task(f"{name} en cours...")
        thread = threading.Thread(target=run, daemon=True)
        self.current_thread = thread
        thread.start()

    def stop_task(self):
        thread = self.current_thread
        if not thread or not thread.is_alive():
            print("Info: aucune tache a arreter.")
            return
        print("-> Demande d'arret en cours...")
        res = ctypes.pythonapi.PyThreadState_SetAsyncExc(
            ctypes.c_long(thread.ident), ctypes.py_object(KeyboardInterrupt)
        )
        if res == 0:
            print("X Impossible d'interrompre la tache.")
        elif res > 1:
            ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(thread.ident), None)
            print("X Interruption annulee (resultat inattendu).")
        else:
            print("-> Signal d'arret envoye.")

    def on_close(self):
        self.stop_task()
        sys.stdout = self.stdout_backup
        sys.stderr = self.stderr_backup
        try:
            dialogue.disconnect_arduino()
        except Exception:
            pass
        self.root.destroy()


def update_robot_on_grid(position, heading):
    global _global_grid_ui
    if _global_grid_ui:
        _global_grid_ui.update_robot_async(position, heading)


def show_notification(title, message, message_type="info"):
    try:
        from tkinter import messagebox
        if message_type == "error":
            messagebox.showerror(title, message)
        elif message_type == "warning":
            messagebox.showwarning(title, message)
        else:
            messagebox.showinfo(title, message)
    except:
        # Fallback si l'UI n'est pas disponible
        pass


def launch_gui():
    root = tk.Tk()
    app = Application(root)
    root.mainloop()
