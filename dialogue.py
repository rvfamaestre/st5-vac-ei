#########################################################################
# CENTRALESUPELEC : ST5 VAC Integration teaching
#
# This script works with the Arduino programmed wih serial_link.ino
# Integrated with line detection and autonomous correction
#
#########################################################################

import os
import random
import serial
import struct
import sys
import time

import cv2
import numpy as np
from collections import deque

# --- État du contrôleur PD pour le suivi de ligne ---
_prev_err = 0.0
_prev_cx = None
_alpha = 0.6  # lissage du centroïde


def reset_steering_state():
    global _prev_err, _prev_cx
    _prev_err = 0.0
    _prev_cx = None


# Rendre les autres modules du projet accessibles lorsque le script est lancé directement
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(CURRENT_DIR, "..", ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.append(PROJECT_ROOT)

if __name__ == "__main__":
    sys.modules.setdefault("dialogue", sys.modules[__name__])

from infrared_utils import get_infrared_distance_cm
from intersections import detect_intersection_with_metrics


def _update_robot_visual(position, heading):
    try:
        # Import dynamique pour éviter les dépendances circulaires
        import menu_ui
        menu_ui.update_robot_on_grid(position, heading)
    except (ImportError, AttributeError):
        # L'UI n'est pas active (mode CLI) - on ignore silencieusement
        pass


# --- Paramètres de détection d'intersection (ajustés pour équilibre détection/faux positifs) ---
INTERSECTION_TRIGGER_RATIO = (
    0.60  # Portion de l'image (60% = zone plus restreinte pour plus de précision)
)
INTERSECTION_FRAMES_REQUIRED = (
    1  # Nombre de frames consécutives (1 seule frame suffisante)
)
INTERSECTION_COOLDOWN_S = (
    2.0  # Délai minimum entre deux détections (2s pour éviter doubles détections)
)
POST_ACTION_PAUSE_S = 0.05  # Pause après action à intersection

# Import de la caméra
try:
    from picamera import PiCamera
    from picamera.array import PiRGBArray

    PICAMERA_AVAILABLE = True
except ImportError:
    print("PiCamera non disponible, mode simulation")
    PICAMERA_AVAILABLE = False


def read_i16(f):
    return struct.unpack("<h", bytearray(f.read(2)))[0]


def read_i32(f):
    return struct.unpack("<l", bytearray(f.read(4)))[0]


def write_i16(f, value):
    f.write(struct.pack("<h", value))


def write_i32(f, value):
    f.write(struct.pack("<l", value))


#######################
# Recherche de Chemin #
#######################

from typing import List, Tuple, Optional

GRID_SIZE = 5
Coord = Tuple[int, int]
Edge = Tuple[Coord, Coord]


def in_bounds(p: Coord) -> bool:
    x, y = p
    return 0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE


def neighbors4(p: Coord) -> List[Coord]:
    x, y = p
    cand = [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]
    return [q for q in cand if in_bounds(q)]


def norm_edge(u: Coord, v: Coord) -> Edge:
    return tuple(sorted((u, v)))


def plan_path_bfs(start, goal, blocked_edges):
    if not (in_bounds(start) and in_bounds(goal)):
        return None
    blocked = {norm_edge(u, v) for (u, v) in blocked_edges}
    q = deque([(start, [start])])
    seen = set([start])

    while q:
        cur, path = q.popleft()
        if cur == goal:
            return path
        for nxt in neighbors4(cur):
            if norm_edge(cur, nxt) in blocked:
                continue
            if nxt in seen:
                continue
            seen.add(nxt)
            q.append((nxt, path + [nxt]))
    return None


turn_left = {"N": "W", "W": "S", "S": "E", "E": "N"}
turn_right = {v: k for k, v in turn_left.items()}


def dir_from_to(a: Coord, b: Coord) -> str:
    ax, ay = a
    bx, by = b
    if bx == ax + 1:
        return "E"
    if bx == ax - 1:
        return "W"
    if by == ay + 1:
        return "N"
    return "S"


def relative_action(heading: str, cur: Coord, nxt: Coord) -> Tuple[str, str]:
    target = dir_from_to(cur, nxt)
    if target == heading:
        return ("S", heading)
    if target == turn_left[heading]:
        return ("L", target)
    if target == turn_right[heading]:
        return ("R", target)
    return ("U", target)


DELTA = {"N": (0, +1), "E": (+1, 0), "S": (0, -1), "W": (-1, 0)}


def advance_start_by_heading(start: Coord, heading: str) -> Optional[Coord]:
    if heading not in DELTA:
        return None
    dx, dy = DELTA[heading]
    nx, ny = start[0] + dx, start[1] + dy
    if in_bounds((nx, ny)):
        return (nx, ny)
    return None


def path_to_actions(path: Optional[List[Coord]], heading_init: str) -> str:
    if not path or len(path) < 2:
        return ""
    h = heading_init
    out = []
    for a, b in zip(path[:-1], path[1:]):
        act, h = relative_action(h, a, b)
        out.append(act)
    return "".join(out)


def plan_and_go_from_inputs(arduino):
    try:
        sx = int(input("Start x (intersection amont) : "))
        sy = int(input("Start y (intersection amont) : "))
        gx = int(input("Goal  x : "))
        gy = int(input("Goal  y : "))
        heading = input("Orientation initiale (N/E/S/W) : ").strip().upper() or "N"

        # Arêtes bloquées
        blocked = []
        n = input("Nombre d'arêtes bloquées (0 si aucune) : ").strip()
        n = int(n) if n else 0
        for i in range(n):
            print(f"Arête bloquée #{i + 1} (entre deux nœuds adjacents)")
            x1 = int(input("  x1 : "))
            y1 = int(input("  y1 : "))
            x2 = int(input("  x2 : "))
            y2 = int(input("  y2 : "))
            blocked.append(((x1, y1), (x2, y2)))

        start = (sx, sy)
        goal = (gx, gy)

        plan_and_go(arduino, start, goal, heading, blocked)
    except Exception as e:
        print(f"\n✗ Erreur de saisie/exécution: {e}")
        import traceback

        traceback.print_exc()


def plan_and_go(
    arduino,
    start,
    goal,
    heading,
    blocked_edges,
    obstacle_callback=None,
    path_callback=None,
):
    blocked = [tuple(map(tuple, edge)) for edge in blocked_edges]
    
    # Afficher le robot à sa position initiale sur la grille
    _update_robot_visual(start, heading)

    # 1) Avancer le start dans la direction du heading (tu es sur l'arête)
    start_effectif = advance_start_by_heading(start, heading)
    
    # Si l'orientation pointe hors de la grille, faire demi-tour d'abord
    if start_effectif is None:
        print("\n" + "=" * 50)
        print("⚠️  ORIENTATION INITIALE INVALIDE")
        print("=" * 50)
        print(f"Position: {start}, Orientation: {heading}")
        print("→ L'orientation pointe vers l'extérieur de la grille!")
        print("→ Correction automatique: DEMI-TOUR")
        print("=" * 50)
        
        # Faire demi-tour pour corriger l'orientation
        make_uturn(arduino)
        time.sleep(0.5)
        
        # Inverser l'orientation
        opposite_heading = {"N": "S", "S": "N", "E": "W", "W": "E"}
        heading = opposite_heading[heading]
        
        # Recalculer start_effectif avec la nouvelle orientation
        start_effectif = advance_start_by_heading(start, heading)
        
        if start_effectif is None:
            print("✗ Erreur: Impossible de corriger l'orientation (problème de configuration).")
            if path_callback:
                path_callback(None)
            return
        
        print(f"✓ Orientation corrigée: {heading}")
        print(f"✓ Start effectif: {start_effectif}")
        time.sleep(1)

    # 2) On ignore l'arête initiale dans les blocs (au cas où elle a été listée)
    initial_edge = norm_edge(start, start_effectif)
    blocked = [e for e in blocked if norm_edge(*e) != initial_edge]

    print(
        f"\n→ Le robot va d'abord tout droit : {start} --({heading})--> {start_effectif}"
    )
    print(
        f"→ Planification BFS depuis {start_effectif} vers {goal} (arêtes bloquées: {len(blocked)})"
    )

    # 3) Planifier depuis l'intersection atteinte après le “tout droit” initial
    path = plan_path_bfs(start_effectif, goal, blocked)
    if not path:
        print("✗ Aucun chemin trouvé depuis le start_effectif (BFS).")
        # Notification dans l'UI
        try:
            import menu_ui
            menu_ui.show_notification(
                "Aucun chemin trouvé",
                f"Impossible de trouver un chemin de {start_effectif} vers {goal}.\n"
                f"Arêtes bloquées: {len(blocked)}\n\n"
                f"Vérifiez les obstacles ou changez le point de départ/arrivée.",
                "error"
            )
        except:
            pass
        if path_callback:
            path_callback(None)
        return

    print("✓ Chemin (depuis start_effectif):", path)

    if path_callback:
        path_nodes = [start] + [tuple(node) for node in path]
        path_callback(path_nodes)

    # 4) Actions à partir de cette première intersection (pas de 'S' initial)
    actions = path_to_actions(path, heading).replace("U", "B")
    if not actions:
        print("Info: déjà à destination après le premier tout droit (0 action).")
    else:
        # Ajouter un 'E' à la fin du chemin pour aller un cran plus loin et faire demi-tour
        actions = actions + "E"

    print("Actions (L=Left, S=Straight, R=Right, B=U-turn, E=End):", actions)

    # Variables pour la boucle de replanification
    current_start = start_effectif
    current_heading = heading
    max_replanning_attempts = 5
    attempt = 0

    # On met tout dans une boucle pour permettre la replanification
    while True:
        attempt += 1

        if attempt > 1:
            # Replanification nécessaire
            print(f"\n{'=' * 50}")
            print(f"REPLANIFICATION #{attempt - 1}")
            print(f"{'=' * 50}")
            print(f"→ Planification BFS depuis {current_start} vers {goal}")
            print(f"→ Arêtes bloquées: {len(blocked)} - {blocked}")

            # Recalculer le chemin
            path = plan_path_bfs(current_start, goal, blocked)
            if not path:
                print("✗ Aucun chemin trouvé. Impossible d'atteindre la destination.")
                print("✗ Toutes les routes sont bloquées!")
                # Notification dans l'UI
                try:
                    import menu_ui
                    menu_ui.show_notification(
                        "Replanification impossible",
                        f"Impossible de trouver un chemin alternatif de {current_start} vers {goal}.\n"
                        f"Arêtes bloquées: {len(blocked)}\n\n"
                        f"Toutes les routes disponibles sont bloquées!",
                        "error"
                    )
                except:
                    pass
                if path_callback:
                    path_callback(None)
                return

            print("✓ Chemin trouvé:", path)

            if path_callback:
                path_callback([tuple(node) for node in path])

            # Recalculer les actions
            actions = path_to_actions(path, current_heading).replace("U", "B")
            if not actions:
                print("Info: déjà à destination (0 action).")
                actions = "E"
            else:
                actions = actions + "E"

            print(f"Actions calculées: {actions}")
            print("(L=Left, S=Straight, R=Right, B=U-turn, E=End)")

        if attempt > max_replanning_attempts:
            print("\n" + "=" * 50)
            print("✗ ÉCHEC: Nombre maximum de replanifications atteint")
            print(
                f"✗ Impossible d'atteindre la destination après {max_replanning_attempts} tentatives"
            )
            print("=" * 50)
            return

        # 5) Lancer l'exécution avec détection d'obstacle
        result = path_following_mode_with_replanning(
            arduino,
            actions,
            path,
            current_heading,
            feedback=True,
            obstacle_stop_distance_cm=25.0,
            obstacle_confirmations_required=3,  # Nécessite 2 détections consécutives
        )

        # Vérifier le résultat de l'exécution
        if result["success"]:
            print("\n" + "=" * 50)
            print("✓✓✓ DESTINATION ATTEINTE AVEC SUCCÈS! ✓✓✓")
            print("=" * 50)
            # Notification de succès dans l'UI
            try:
                import menu_ui
                menu_ui.show_notification(
                    "Destination atteinte !",
                    f"Le robot a atteint la destination {goal} avec succès !",
                    "info"
                )
            except:
                pass
            return

        elif result["obstacle_detected"]:
            # Un obstacle a été détecté - préparer la replanification
            obstacle_edge = result["blocked_edge"]
            obstacle_position = result["current_position"]
            obstacle_heading = result["current_heading"]

            print("\n" + "=" * 50)
            print("⚠️  GESTION DE L'OBSTACLE")
            print("=" * 50)
            print(f"→ Position lors de la détection: {obstacle_position}")
            print(f"→ Orientation lors de la détection: {obstacle_heading}")
            print(f"→ Arête bloquée: {obstacle_edge}")
            
            # Note: Notification désactivée pour ne pas bloquer le programme
            # Le message apparaît dans le journal de log uniquement

            # Ajouter l'arête bloquée à la liste
            if obstacle_edge:
                # Normaliser l'arête
                normalized_edge = (
                    norm_edge(*obstacle_edge)
                    if isinstance(obstacle_edge[0], tuple)
                    else obstacle_edge
                )
                if normalized_edge not in blocked:
                    blocked.append(normalized_edge)
                    print(f"→ Arête {normalized_edge} ajoutée aux obstacles connus")
                    if obstacle_callback:
                        obstacle_callback(normalized_edge)
                else:
                    print(f"→ Arête {normalized_edge} déjà dans la liste des obstacles")

            # Faire demi-tour
            print("\n→ Exécution d'un DEMI-TOUR...")
            make_uturn(arduino)
            time.sleep(0.5)

            if path_callback:
                path_callback(None)

            # Mettre à jour la position et l'orientation pour la replanification
            current_start = obstacle_position

            # Inverser l'orientation après le demi-tour
            opposite_heading = {"N": "S", "S": "N", "E": "W", "W": "E"}
            current_heading = opposite_heading[obstacle_heading]
            
            # Mettre à jour la position visuelle du robot
            _update_robot_visual(current_start, current_heading)

            print(f"→ Nouvelle position de départ: {current_start}")
            print(f"→ Nouvelle orientation: {current_heading}")
            print("\n→ Recherche d'un chemin alternatif...\n")
            time.sleep(1)
            # Continue la boucle pour replanifier

        else:
            # Arrêt inattendu (Ctrl+C ou erreur)
            print("\n⚠️  Arrêt du programme (interruption utilisateur ou erreur)")
            return


############################################
# Fonctions de vision et traitement d'image
############################################

# Configuration de la caméra
resolution_target = (160, 128)


def init_camera():
    if not PICAMERA_AVAILABLE:
        return None, None, None

    camera = PiCamera(sensor_mode=2)
    camera.resolution = resolution_target
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=camera.resolution)
    frame_source = camera.capture_continuous(
        rawCapture, format="bgr", use_video_port=True
    )

    return camera, rawCapture, frame_source


def capture_image(frame_source, rawCapture):
    if frame_source is None:
        return None

    image = next(frame_source).array
    rawCapture.truncate(0)
    return image


def detect_line(image, feedback=False):
    if image is None:
        return None, None

    h, w = image.shape[:2]

    # FILTRAGE: Ne garder que la partie basse de l'image (éviter les murs)
    # On garde seulement les 60% inférieurs de l'image
    roi_height = int(h * 0.6)
    roi_start = h - roi_height
    image_roi = image[roi_start:h, 0:w]

    # Prétraitement: flou pour réduire le bruit
    blur = cv2.blur(image_roi, (5, 5))

    # Seuillage pour isoler les zones blanches
    ret, thresh1 = cv2.threshold(blur, 168, 255, cv2.THRESH_BINARY)

    # Conversion en HSV
    hsv = cv2.cvtColor(thresh1, cv2.COLOR_RGB2HSV)

    # Définition de la plage de blanc en HSV
    lower_white = np.array([0, 0, 168])
    upper_white = np.array([172, 111, 255])

    # Création du masque
    mask = cv2.inRange(hsv, lower_white, upper_white)

    # Suppression du bruit avec morphologie
    kernel_erode = np.ones((6, 6), np.uint8)
    eroded_mask = cv2.erode(mask, kernel_erode, iterations=1)
    kernel_dilate = np.ones((4, 4), np.uint8)
    dilated_mask = cv2.dilate(eroded_mask, kernel_dilate, iterations=1)

    # Détection des contours
    contours, hierarchy = cv2.findContours(
        dilated_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
    )

    # Note: cv2.imshow() désactivé car il ne fonctionne pas correctement depuis un thread
    # (provoque des erreurs X Window System)
    # if feedback:
    #     im_debug = cv2.drawContours(image.copy(), contours, -1, (0, 255, 0), 2)
    #     cv2.imshow("Contours détectés", im_debug)
    #     cv2.waitKey(1)

    # Tri par aire (garder seulement le plus grand)
    if len(contours) > 0:
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:1]
        M = cv2.moments(contours[0])

        if M["m00"] != 0:
            # Calcul du centroïde dans la ROI
            cx = int(M["m10"] / M["m00"])
            cy = (
                int(M["m01"] / M["m00"]) + roi_start
            )  # Ajuster y par rapport à l'image complète

            # Ne pas afficher le centroïde à chaque frame
            # if feedback:
            #     print(f"Centroïde détecté à: ({cx}, {cy})")

            return cx, cy

    # if feedback:
    #     print("Aucune ligne détectée")

    return None, None


def compute_steering_command(cx, cy, image_width):
    global _prev_err, _prev_cx, _alpha
    import numpy as np

    # — Paramètres à ajuster —
    base = 200  # vitesse de base
    vmin, vmax = 60, 250
    soft_deadband = 0.015  # 1.5% de la largeur : plus petit => réagit plus tôt
    Kp_small = 1.6  # gain P pour petites erreurs (réaction précoce)
    Kp_big = 1.1  # gain P nominal pour grandes erreurs
    Kd = 0.18  # dérivé (amortit)
    turn_gain = 120  # autorité de braquage (100→120 ou 130 pour plus tôt/fort)
    shape_gain = (
        1.2  # courbure de la mise en forme (tanh); >1 => pousse les petites erreurs
    )

    # Perte de ligne : recherche lente
    if cx is None:
        return 200, -200

    # Lissage du centroïde
    if _prev_cx is None:
        cx_s = cx
    else:
        cx_s = int(_alpha * _prev_cx + (1 - _alpha) * cx)
    _prev_cx = cx_s

    center = image_width / 2.0
    err = (cx_s - center) / center  # [-1..1]

    # ----- Deadband "doux" : on compresse au lieu de couper net -----
    # Si l'erreur est très petite, on la “déplafonne” doucement pour obtenir une correction plus tôt.
    if abs(err) < soft_deadband:
        # Remap l’erreur dans le deadband pour lui donner du poids (expo 1.5 = pré-accentuation)
        if soft_deadband > 1e-6:
            err_eff = np.sign(err) * (abs(err) / soft_deadband) ** 1.5 * soft_deadband
        else:
            err_eff = err
    else:
        err_eff = err

    # ----- Gain piecewise : plus de Kp quand l’erreur est petite -----
    Kp = Kp_small if abs(err_eff) < 0.10 else Kp_big

    # ----- Mise en forme non linéaire (tanh) : boost des petites erreurs, plafonne les grandes -----
    e_shaped = np.tanh(err_eff * shape_gain) / np.tanh(shape_gain)  # reste dans [-1..1]

    # PD
    d_err = err_eff - _prev_err
    _prev_err = err_eff
    turn = Kp * e_shaped + Kd * d_err

    # Autorité de braquage
    diff = int(np.clip(turn, -1, 1) * turn_gain)

    left = int(np.clip(base - diff, vmin, vmax))
    right = int(np.clip(base + diff, vmin, vmax))
    return left, right


def send_motor_command(arduino, left_speed, right_speed):
    # Vider le buffer de réception avant d'envoyer la commande
    arduino.reset_input_buffer()

    # Protocole: 'C' + 4 × int16 (comme carAdvance dans test_moteurs.py)
    arduino.write(b"C")
    write_i16(arduino, int(left_speed))  # Moteur gauche
    write_i16(arduino, int(right_speed))  # Moteur droit
    write_i16(arduino, 0)  # Argument 3 (non utilisé)
    write_i16(arduino, 0)  # Argument 4 (non utilisé)
    arduino.flush()  # Force l'envoi des données

    # Attente de l'acquittement
    rep = b""
    timeout_count = 0
    max_timeout = 10

    while rep == b"" and timeout_count < max_timeout:
        rep = arduino.readline()
        timeout_count += 1
        if rep == b"":
            time.sleep(0.001)  # Petite pause si pas de réponse

    if rep:
        response = rep.decode().strip()
        if "ER" in response:
            print(f"⚠️  Arduino: {response}")
        # else:
        #     print(f"✓ Arduino: {response}")  # Désactivé pour éviter le spam
    else:
        print("⚠️  Timeout: Pas de réponse de l'Arduino")


############################################
# Gestion des intersections
############################################

INTERSECTION_ACTIONS = ("left", "straight", "right", "back", "end")
ACTION_LABELS = {
    "left": "tourner à gauche",
    "straight": "continuer tout droit",
    "right": "tourner à droite",
    "back": "faire demi-tour",
    "end": "aller un cran plus loin et faire demi-tour",
}


def choose_random_intersection_action() -> str:
    print("\n" + "=" * 50)
    print("INTERSECTION DÉTECTÉE")
    print("=" * 50)
    print("Je me demande: où tourner ? Options: gauche, tout droit, droite, demi-tour.")
    action = random.choice(INTERSECTION_ACTIONS)
    print(f"→ Décision aléatoire: {ACTION_LABELS[action]}")
    return action


def perform_intersection_action(
    arduino, action, straight_speed=230, straight_duration=0.7
):
    if action not in ACTION_LABELS:
        print(f"Action inconnue: {action}")
        return

    if action == "left":
        make_left_turn(arduino)
    elif action == "right":
        make_right_turn(arduino)
    elif action == "back":
        make_uturn(arduino)
    elif action == "straight":
        print("intersection traverseee tout droit")
    elif action == "end":
        True == True
    else:
        print(f"Action inconnue: {action}")


############################################
# Fonction de suivi de ligne autonome
############################################


def autonomous_line_following(
    arduino,
    duration=60,
    feedback=True,
    enable_intersection_decision=True,
    enable_obstacle_stop=True,
    obstacle_stop_distance_cm=30.0,
):
    print("\n" + "=" * 50)
    print("DÉMARRAGE DU MODE SUIVI DE LIGNE AUTONOME")
    print("=" * 50)
    print("Appuyez sur Ctrl+C pour arrêter")
    print()

    # Initialisation de la caméra
    camera, rawCapture, frame_source = init_camera()

    if camera is None:
        print("Erreur: Impossible d'initialiser la caméra")
        return

    print("Caméra initialisée")
    time.sleep(1)

    start_time = time.time()
    frame_count = 0
    intersection_frame_counter = 0
    last_intersection_time = -float("inf")
    intersection_trigger_ratio = INTERSECTION_TRIGGER_RATIO
    intersection_frames_required = INTERSECTION_FRAMES_REQUIRED
    intersection_decision_cooldown_s = INTERSECTION_COOLDOWN_S
    post_action_pause_s = POST_ACTION_PAUSE_S
    latest_detection = None

    reset_steering_state()

    try:
        while True:
            # Vérifier la durée
            if duration > 0 and (time.time() - start_time) > duration:
                print(f"\nDurée écoulée ({duration}s)")
                break

            # Capture d'image
            image = capture_image(frame_source, rawCapture)

            if image is None:
                print("Erreur de capture d'image")
                time.sleep(0.1)
                continue

            frame_count += 1

            # Détection de la ligne
            cx, cy = detect_line(image, feedback=feedback)

            # Calcul de la commande de direction
            # reset_steering_state()
            left_speed, right_speed = compute_steering_command(cx, cy, image.shape[1])

            # Gestion de l'intersection et de la décision aléatoire
            intersection_triggered = False
            if enable_intersection_decision:
                now = time.time()
                cooldown_active = (
                    now - last_intersection_time
                ) < intersection_decision_cooldown_s

                if cooldown_active:
                    intersection_frame_counter = 0
                    latest_detection = None
                else:
                    detection = detect_intersection_with_metrics(
                        image,
                        feedback=False,  # Désactiver le feedback de intersections.py
                    )
                    latest_detection = detection
                    if detection.coordinates is not None:
                        ix, iy = detection.coordinates
                        trigger_y = int(image.shape[0] * intersection_trigger_ratio)
                        if iy >= trigger_y:
                            intersection_frame_counter += 1
                            if feedback:
                                print(
                                    f"[Intersection] Frame {intersection_frame_counter}/{intersection_frames_required} "
                                    f"({ix}, {iy}) - v={detection.vertical_density:.2f}, "
                                    f"h={detection.horizontal_density:.2f}, "
                                    f"w={detection.width_ratio:.2f}, white={detection.white_ratio:.3f}"
                                )
                        else:
                            if feedback and intersection_frame_counter > 0:
                                print(
                                    f"[Intersection] ✗ Trop haut: iy={iy} < trigger_y={trigger_y} - RESET compteur"
                                )
                            intersection_frame_counter = 0
                    else:
                        if feedback and intersection_frame_counter > 0:
                            print(
                                f"[Intersection] ✗ Perdu (pas détecté) - RESET compteur"
                            )
                        intersection_frame_counter = 0

                    if intersection_frame_counter >= intersection_frames_required:
                        intersection_triggered = True

            if intersection_triggered:
                if feedback:
                    print(
                        "[Intersection] Intersection confirmée - arrêt complet puis décision aléatoire."
                    )
                send_motor_command(arduino, 0, 0)
                time.sleep(0.1)
                send_motor_command(arduino, 250, 250)
                time.sleep(0.2)
                send_motor_command(arduino, 0, 0)
                time.sleep(0.1)

                action = choose_random_intersection_action()
                perform_intersection_action(arduino, action)

                last_intersection_time = time.time()
                intersection_frame_counter = 0
                latest_detection = None
                time.sleep(post_action_pause_s)
                continue

            obstacle_detected = False
            obstacle_distance = None
            if enable_obstacle_stop:
                distance_cm = get_infrared_distance_cm(arduino)
                if distance_cm is not None and distance_cm <= obstacle_stop_distance_cm:
                    obstacle_detected = True
                    obstacle_distance = distance_cm

            if obstacle_detected:
                if feedback:
                    print(f"obstacle détecté à {distance_cm} cm")
                send_motor_command(arduino, 230, -230)
                continue

            # Envoi de la commande aux moteurs
            send_motor_command(arduino, left_speed, right_speed)

            # Affichage des statistiques (seulement toutes les 100 frames)
            if frame_count % 100 == 0:
                fps = frame_count / (time.time() - start_time)
                print(f"[Stats] Frames: {frame_count} | FPS: {fps:.1f}")

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n\nArrêt demandé par l'utilisateur")
    finally:
        # Arrêt des moteurs
        print("Arrêt des moteurs...")
        send_motor_command(arduino, 0, 0)

        # Fermeture de la caméra
        if camera is not None:
            camera.close()

        print("✓ Caméra fermée")
        print("=" * 50)


############################################
# Fonction de déplacement (virages, demi-tours)
#############################################


def make_uturn(arduino):
    print("\n" + "=" * 50)
    print("DEMI-TOUR EN COURS")
    print("=" * 50)

    # Démarrer la rotation (moteur gauche en arrière, moteur droit en avant)
    send_motor_command(arduino, -250, 250)

    time.sleep(1.34)

    # Arrêt des moteurs
    send_motor_command(arduino, 0, 0)

    print("✓ Demi-tour terminé")
    print("=" * 50 + "\n")


def make_left_turn(arduino):
    print("\n" + "=" * 50)
    print("VIRAGE À GAUCHE EN COURS")
    print("=" * 50)

    # Démarrer la rotation (moteur gauche en arrière, moteur droit en avant)
    send_motor_command(arduino, 250, -250)

    time.sleep(0.62)

    # Arrêt des moteurs
    send_motor_command(arduino, 0, 0)

    print("Virage à gauche terminé")
    print("=" * 50 + "\n")


def make_right_turn(arduino):
    print("\n" + "=" * 50)
    print("VIRAGE À DROITE EN COURS")
    print("=" * 50)

    # Démarrer la rotation (moteur gauche en avant, moteur droit en arrière)
    send_motor_command(arduino, -250, 250)

    time.sleep(0.62)

    # Arrêt des moteurs
    send_motor_command(arduino, 0, 0)

    print("Virage à droite terminé")
    print("=" * 50 + "\n")


############################################
# Mode de suivi de trajet prédéfini
############################################


def path_following_mode_with_replanning(
    arduino,
    action_path,
    coord_path,
    initial_heading,
    feedback=True,
    obstacle_stop_distance_cm=25.0,
    obstacle_confirmations_required=3,
):
    print("\n" + "=" * 50)
    print("DÉMARRAGE DU MODE SUIVI DE TRAJET AVEC DÉTECTION D'OBSTACLES")
    print("=" * 50)
    print(f"Trajet à suivre: {action_path}")
    print(f"Chemin de coordonnées: {coord_path}")
    print(f"Nombre d'intersections: {len(action_path)}")
    print(f"Orientation initiale: {initial_heading}")
    print(
        f"Détection d'obstacles: distance ≤ {obstacle_stop_distance_cm} cm, {obstacle_confirmations_required} confirmations requises"
    )
    print("Appuyez sur Ctrl+C pour arrêter")
    print()

    # Validation du chemin
    valid_commands = set("LRSBE")
    action_path = action_path.upper().strip()
    for char in action_path:
        if char not in valid_commands:
            print(f"Erreur: caractère invalide '{char}' dans le chemin")
            return {"success": False, "obstacle_detected": False}

    # Initialisation de la caméra
    camera, rawCapture, frame_source = init_camera()

    if camera is None:
        print("Erreur: Impossible d'initialiser la caméra")
        return {"success": False, "obstacle_detected": False}

    print("Caméra initialisée")
    time.sleep(1)

    # Variables de contrôle
    start_time = time.time()
    frame_count = 0
    intersection_frame_counter = 0
    last_intersection_time = -float("inf")
    intersection_trigger_ratio = INTERSECTION_TRIGGER_RATIO
    intersection_frames_required = INTERSECTION_FRAMES_REQUIRED
    intersection_decision_cooldown_s = INTERSECTION_COOLDOWN_S
    post_action_pause_s = POST_ACTION_PAUSE_S

    # Index pour suivre la progression dans le trajet
    path_index = 0

    # Tracking de la position et orientation
    # IMPORTANT: current_position = dernière intersection physiquement franchie
    # Au départ, on est à coord_path[0] (première intersection du chemin)
    current_heading = initial_heading
    current_position = coord_path[0] if coord_path else None

    # Variables pour la détection d'obstacles avec confirmation
    obstacle_confirmation_count = 0
    # obstacle_confirmations_required est passé en paramètre

    if feedback:
        print(
            f"Démarrage - Position initiale: {current_position}, Orientation: {current_heading}"
        )

    try:
        while path_index < len(action_path):
            # Capture d'image
            image = capture_image(frame_source, rawCapture)

            if image is None:
                print("Erreur de capture d'image")
                time.sleep(0.1)
                continue

            frame_count += 1

            # Détection de la ligne
            cx, cy = detect_line(image, feedback=feedback)

            # Calcul de la commande de direction
            reset_steering_state()
            left_speed, right_speed = compute_steering_command(cx, cy, image.shape[1])

            # Gestion de la détection d'intersection
            intersection_triggered = False
            now = time.time()
            cooldown_active = (
                now - last_intersection_time
            ) < intersection_decision_cooldown_s

            if cooldown_active:
                intersection_frame_counter = 0
            else:
                detection = detect_intersection_with_metrics(
                    image,
                    feedback=False,
                )
                if detection.coordinates is not None:
                    ix, iy = detection.coordinates
                    trigger_y = int(image.shape[0] * intersection_trigger_ratio)
                    if iy >= trigger_y:
                        intersection_frame_counter += 1
                        if feedback:
                            print(
                                f"[Intersection] Frame {intersection_frame_counter}/{intersection_frames_required}"
                            )
                    else:
                        if feedback and intersection_frame_counter > 0:
                            print(f"[Intersection] ✗ Trop haut - RESET compteur")
                        intersection_frame_counter = 0
                else:
                    if feedback and intersection_frame_counter > 0:
                        print(f"[Intersection] ✗ Perdu - RESET compteur")
                    intersection_frame_counter = 0

                if intersection_frame_counter >= intersection_frames_required:
                    intersection_triggered = True

            # Exécution de l'action prédéfinie à l'intersection
            if intersection_triggered:
                command = action_path[path_index]
                print("\n" + "=" * 50)
                print(f"INTERSECTION {path_index + 1}/{len(action_path)} DÉTECTÉE")
                print(f"Commande à exécuter: {command}")
                print("=" * 50)

                time.sleep(0.2)

                # Conversion de la commande en action
                action_map = {
                    "L": "left",
                    "R": "right",
                    "S": "straight",
                    "B": "back",
                    "E": "end",
                }
                action = action_map[command]

                print(f"→ Action: {ACTION_LABELS[action]}")
                perform_intersection_action(arduino, action)

                # Mettre à jour l'orientation après l'action
                if command == "L":
                    current_heading = turn_left[current_heading]
                elif command == "R":
                    current_heading = turn_right[current_heading]
                elif command == "B":
                    opposite = {"N": "S", "S": "N", "E": "W", "W": "E"}
                    current_heading = opposite[current_heading]
                # 'S' et 'E' ne changent pas l'orientation

                # Mise à jour des variables de contrôle
                last_intersection_time = time.time()
                intersection_frame_counter = 0

                # IMPORTANT: Incrémenter path_index APRÈS l'action
                path_index += 1

                # Mettre à jour current_position = l'intersection qu'on vient de franchir
                # On vient de franchir l'intersection à coord_path[path_index - 1]
                # Donc current_position doit être mise à jour vers cette intersection
                if path_index - 1 < len(coord_path):
                    current_position = coord_path[path_index - 1]
                
                # Mettre à jour la position visuelle du robot sur la grille
                _update_robot_visual(current_position, current_heading)

                # Afficher où on est maintenant
                if path_index < len(coord_path):
                    # On est maintenant sur l'arête vers la prochaine intersection
                    if feedback:
                        print(f"→ Position mise à jour: {current_position}")
                        print(
                            f"→ On est maintenant sur l'arête de {current_position} vers {coord_path[path_index]}"
                        )
                else:
                    # On a franchi la dernière intersection
                    if feedback:
                        print(
                            f"→ Position mise à jour: {current_position}, orientation: {current_heading}"
                        )

                # Affichage de la progression
                if path_index < len(action_path):
                    print(f"\n✓ Intersection {path_index}/{len(action_path)} franchie")
                    print(f"Prochaine commande: {action_path[path_index]}")
                else:
                    print(f"\n✓ Toutes les intersections ont été franchies!")
                    print("Trajet terminé!")

                time.sleep(post_action_pause_s)
                continue

            # Détection d'obstacle avec CONFIRMATION MULTIPLE (éviter les faux positifs)
            distance_cm = get_infrared_distance_cm(arduino)

            if distance_cm is not None and distance_cm <= obstacle_stop_distance_cm:
                # Obstacle potentiellement détecté - incrémenter le compteur
                obstacle_confirmation_count += 1

                if (
                    feedback
                    and obstacle_confirmation_count < obstacle_confirmations_required
                ):
                    print(
                        f"[Obstacle?] Détection {obstacle_confirmation_count}/{obstacle_confirmations_required} à {distance_cm:.1f} cm"
                    )

                # Vérifier si on a assez de confirmations consécutives
                if obstacle_confirmation_count >= obstacle_confirmations_required:
                    print(
                        f"\n⚠️  OBSTACLE CONFIRMÉ après {obstacle_confirmations_required} détections consécutives!"
                    )
                    print(f"   Dernière distance mesurée: {distance_cm:.1f} cm")

                    # Arrêter les moteurs immédiatement
                    send_motor_command(arduino, 0, 0)

                    # Fermeture de la caméra
                    if camera is not None:
                        camera.close()

                    # CORRECTION COMPLÈTE: Déterminer l'arête bloquée
                    # Le robot est physiquement sur l'arête ENTRE:
                    # - current_position (dernière intersection franchie)
                    # - coord_path[path_index] (prochaine intersection à atteindre)

                    if current_position and path_index < len(coord_path):
                        next_position = coord_path[path_index]
                        blocked_edge = norm_edge(current_position, next_position)

                        if feedback:
                            print(f"\nDEBUG Détection obstacle:")
                            print(
                                f"  - current_position (dernière franchie): {current_position}"
                            )
                            print(f"  - path_index: {path_index}")
                            print(f"  - coord_path: {coord_path}")
                            print(f"  - next_position (cible): {next_position}")
                            print(f"  - Orientation: {current_heading}")
                            print(f"  → Arête bloquée: {blocked_edge}")
                    else:
                        # Cas de secours: calculer basé sur l'orientation
                        next_pos = advance_start_by_heading(
                            current_position, current_heading
                        )
                        blocked_edge = (
                            norm_edge(current_position, next_pos)
                            if (current_position and next_pos)
                            else None
                        )

                        if feedback:
                            print(f"\nDEBUG Détection obstacle (méthode de secours):")
                            print(f"  - Position actuelle: {current_position}")
                            print(f"  - Orientation: {current_heading}")
                            print(f"  - Prochaine position calculée: {next_pos}")
                            print(f"  → Arête bloquée: {blocked_edge}")

                    return {
                        "success": False,
                        "obstacle_detected": True,
                        "blocked_edge": blocked_edge,
                        "current_position": current_position,
                        "current_heading": current_heading,
                        "path_index": path_index,
                    }
            else:
                # Pas d'obstacle ou hors de portée - réinitialiser le compteur
                if obstacle_confirmation_count > 0:
                    if feedback:
                        print(
                            f"[Obstacle?] Fausse alerte - RESET compteur (était à {obstacle_confirmation_count})"
                        )
                obstacle_confirmation_count = 0

            # Envoi de la commande aux moteurs
            send_motor_command(arduino, left_speed, right_speed)

            # Affichage des statistiques (seulement toutes les 100 frames)
            if frame_count % 100 == 0:
                fps = frame_count / (time.time() - start_time)
                remaining = len(action_path) - path_index
                print(
                    f"[Stats] Frames: {frame_count} | FPS: {fps:.1f} | Intersections restantes: {remaining}"
                )

            time.sleep(0.02)

        # Trajet terminé avec succès
        print("\n" + "=" * 50)
        print("TRAJET TERMINÉ AVEC SUCCÈS!")
        print("=" * 50)
        print(f"Temps total: {time.time() - start_time:.1f}s")
        print("=" * 50)

        # Arrêt des moteurs
        send_motor_command(arduino, 0, 0)

        # Fermeture de la caméra
        if camera is not None:
            camera.close()

        return {"success": True, "obstacle_detected": False}

    except KeyboardInterrupt:
        print("\n\nArrêt demandé par l'utilisateur")
        send_motor_command(arduino, 0, 0)
        if camera is not None:
            camera.close()
        return {"success": False, "obstacle_detected": False}
    except Exception as e:
        print(f"\n⚠️  Erreur: {e}")
        import traceback

        traceback.print_exc()
        send_motor_command(arduino, 0, 0)
        if camera is not None:
            camera.close()
        return {"success": False, "obstacle_detected": False}


def path_following_mode(
    arduino,
    path,
    feedback=True,
    enable_obstacle_stop=True,
    obstacle_stop_distance_cm=25.0,
):
    print("\n" + "=" * 50)
    print("DÉMARRAGE DU MODE SUIVI DE TRAJET PRÉDÉFINI")
    print("=" * 50)
    print(f"Trajet à suivre: {path}")
    print(f"Nombre d'intersections: {len(path)}")
    print("Appuyez sur Ctrl+C pour arrêter")
    print()

    # Validation du chemin
    valid_commands = set("LRSBE")
    path = path.upper().strip()
    for char in path:
        if char not in valid_commands:
            print(f"Erreur: caractère invalide '{char}' dans le chemin")
            print(
                f"Caractères valides: L (gauche), R (droite), S (tout droit), B (demi-tour), E (end - aller plus loin et demi-tour)"
            )
            return

    # Initialisation de la caméra
    camera, rawCapture, frame_source = init_camera()

    if camera is None:
        print("Erreur: Impossible d'initialiser la caméra")
        return

    print("Caméra initialisée")
    time.sleep(1)

    # Variables de contrôle
    start_time = time.time()
    frame_count = 0
    intersection_frame_counter = 0
    last_intersection_time = -float("inf")
    intersection_trigger_ratio = INTERSECTION_TRIGGER_RATIO
    intersection_frames_required = INTERSECTION_FRAMES_REQUIRED
    intersection_decision_cooldown_s = INTERSECTION_COOLDOWN_S
    post_action_pause_s = POST_ACTION_PAUSE_S
    latest_detection = None

    # Index pour suivre la progression dans le trajet
    path_index = 0

    try:
        while path_index < len(path):
            # Capture d'image
            image = capture_image(frame_source, rawCapture)

            if image is None:
                print("Erreur de capture d'image")
                time.sleep(0.1)
                continue

            frame_count += 1

            # Détection de la ligne
            cx, cy = detect_line(image, feedback=feedback)

            # Calcul de la commande de direction
            reset_steering_state()
            left_speed, right_speed = compute_steering_command(cx, cy, image.shape[1])

            # Gestion de la détection d'intersection
            intersection_triggered = False
            now = time.time()
            cooldown_active = (
                now - last_intersection_time
            ) < intersection_decision_cooldown_s

            if cooldown_active:
                intersection_frame_counter = 0
                latest_detection = None
            else:
                detection = detect_intersection_with_metrics(
                    image,
                    feedback=False,  # Désactiver le spam des messages
                )
                latest_detection = detection
                if detection.coordinates is not None:
                    ix, iy = detection.coordinates
                    trigger_y = int(image.shape[0] * intersection_trigger_ratio)
                    if iy >= trigger_y:
                        intersection_frame_counter += 1
                        if feedback:
                            print(
                                f"[Intersection] Frame {intersection_frame_counter}/{intersection_frames_required} "
                                f"({ix}, {iy}) - v={detection.vertical_density:.2f}, "
                                f"h={detection.horizontal_density:.2f}, "
                                f"w={detection.width_ratio:.2f}, white={detection.white_ratio:.3f}"
                            )
                    else:
                        if feedback and intersection_frame_counter > 0:
                            print(
                                f"[Intersection] ✗ Trop haut: iy={iy} < trigger_y={trigger_y} - RESET compteur"
                            )
                        intersection_frame_counter = 0
                else:
                    if feedback and intersection_frame_counter > 0:
                        print(f"[Intersection] ✗ Perdu (pas détecté) - RESET compteur")
                    intersection_frame_counter = 0

                if intersection_frame_counter >= intersection_frames_required:
                    intersection_triggered = True

            # Exécution de l'action prédéfinie à l'intersection
            if intersection_triggered:
                command = path[path_index]
                print("\n" + "=" * 50)
                print(f"INTERSECTION {path_index + 1}/{len(path)} DÉTECTÉE")
                print(f"Commande à exécuter: {command}")
                print("=" * 50)

                time.sleep(0.2)

                # Conversion de la commande en action
                action_map = {
                    "L": "left",
                    "R": "right",
                    "S": "straight",
                    "B": "back",
                    "E": "end",
                }
                action = action_map[command]

                print(f"→ Action: {ACTION_LABELS[action]}")
                perform_intersection_action(arduino, action)

                # Mise à jour des variables de contrôle
                last_intersection_time = time.time()
                intersection_frame_counter = 0
                latest_detection = None
                path_index += 1

                # Affichage de la progression
                if path_index < len(path):
                    print(f"\n✓ Intersection {path_index}/{len(path)} franchie")
                    print(f"Prochaine commande: {path[path_index]}")
                else:
                    print(f"\n✓ Toutes les intersections ont été franchies!")
                    print("Trajet terminé!")

                time.sleep(post_action_pause_s)
                continue

            # Détection d'obstacle
            obstacle_detected = False
            obstacle_distance = None
            if enable_obstacle_stop:
                distance_cm = get_infrared_distance_cm(arduino)
                if distance_cm is not None and distance_cm <= obstacle_stop_distance_cm:
                    obstacle_detected = True
                    obstacle_distance = distance_cm

            if obstacle_detected:
                if feedback:
                    print(
                        f"⚠️  Obstacle détecté à {obstacle_distance:.1f} cm - rotation"
                    )
                send_motor_command(arduino, 230, -230)
                continue

            # Envoi de la commande aux moteurs
            send_motor_command(arduino, left_speed, right_speed)

            # Affichage des statistiques (seulement toutes les 100 frames)
            if frame_count % 100 == 0:
                fps = frame_count / (time.time() - start_time)
                remaining = len(path) - path_index
                print(
                    f"[Stats] Frames: {frame_count} | FPS: {fps:.1f} | Intersections restantes: {remaining}"
                )

            time.sleep(0.02)

        # Trajet terminé
        print("\n" + "=" * 50)
        print("TRAJET TERMINÉ AVEC SUCCÈS!")
        print("=" * 50)
        print(f"Toutes les {len(path)} intersections ont été franchies")
        print(f"Temps total: {time.time() - start_time:.1f}s")
        print("=" * 50)

    except KeyboardInterrupt:
        print("\n\nArrêt demandé par l'utilisateur")
        print(f"Progression: {path_index}/{len(path)} intersections franchies")
    finally:
        # Arrêt des moteurs
        print("Arrêt des moteurs...")
        send_motor_command(arduino, 0, 0)

        # Fermeture de la caméra
        if camera is not None:
            camera.close()

        print("✓ Caméra fermée")
        print("=" * 50)


############################################
# Fonction de dialogue direct avec l'arduino
#############################################


def DialArduino():
    while True:
        print("")
        print("Dialogue direct avec l'arduino")
        cma = input("Tapez votre commande arduino (Q pour finir) ")
        if cma == "Q":
            send_motor_command(arduino, 0, 0)
            break
        if cma == "u":
            make_uturn(arduino)
            continue
        if cma == "l":
            make_left_turn(arduino)
            continue
        if cma == "r":
            make_right_turn(arduino)
            continue
        if cma == "f":
            send_motor_command(arduino, 255, 255)
            time.sleep(5)
            send_motor_command(arduino, 0, 0)
        if cma == "b":
            send_motor_command(arduino, -255, -255)
            time.sleep(5)
            send_motor_command(arduino, 0, 0)
        if cma != "":
            arduino.write(cma.encode("utf-8"))
            time.sleep(0.01)
            rep = arduino.readline()  # on lit le message de réponse
            while rep == b"":  # On attend d'avoir une vraie réponse
                rep = arduino.readline()  # on lit le message de réponse
            # print(rep)
            print(rep.decode())
            while (
                arduino.inWaiting() > 0
            ):  # tant qu'on a des messages dans le buffer de retour
                rep = arduino.readline()  # on lit le message de réponse
                print(rep.decode())


def AttAcquit():
    rep = b""
    while rep == b"":  # attend l'acquitement du B2
        rep = arduino.readline()
    # print(rep.decode())


############################################################
# Gestion de la liaison série avec l'arduino

arduino = None


def initialize_arduino(port="/dev/ttyACM0", baudrate=115200, timeout=0.1):
    global arduino
    if arduino is not None and arduino.is_open:
        return arduino

    arduino = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)

    # Vider le buffer série
    rep = b" "
    while rep != b"":
        rep = arduino.readline()

    print("Connection à l'arduino")
    time.sleep(2)  # on attend 2s pour que la carte soit initialisée

    arduino.write(b"A22")  # demande de connection en mode binaire complet (commode=2)
    rep = arduino.readline()
    if len(rep.split()) > 0 and rep.split()[0] == b"OK":
        print(rep.decode())

    return arduino


def disconnect_arduino():
    global arduino
    if arduino is None:
        return
    try:
        if arduino.is_open:
            arduino.write(b"a")  # deconnection de la carte
            arduino.close()  # fermeture de la liaison série
            print("Fin de programme")
    finally:
        arduino = None


def run_cli_menu():
    arduino_conn = initialize_arduino()
    if arduino_conn is None:
        return

    while True:
        print("\n" + "=" * 50)
        print("MENU PRINCIPAL")
        print("=" * 50)
        print("1. Dialogue direct avec Arduino")
        print("2. Mode suivi de ligne autonome")
        print("3. Mode suivi de trajet prédéfini")
        print("4. Naviguer de A à B")
        print("Q. Quitter")
        print("=" * 50)

        choix = input("Votre choix: ").strip().upper()

        if choix == "1":
            print(
                "to quit : Q, to make uturn : u, left turn : l, right turn : r, forward : f, backwards : b"
            )
            DialArduino()
        elif choix == "2":
            duree = input("Durée du suivi (en secondes, 0 pour infini): ").strip()
            try:
                duree = int(duree)
            except:
                duree = 60
            autonomous_line_following(arduino_conn, duration=duree, feedback=True)
        elif choix == "3":
            print("\nMode suivi de trajet prédéfini")
            print("Instructions valides:")
            print("  L = tourner à gauche")
            print("  R = tourner à droite")
            print("  S = continuer tout droit")
            print("  B = faire demi-tour")
            path = input("Entrez le trajet (ex: LRRLRL): ").strip().upper()
            if path:
                path_following_mode(arduino_conn, path, feedback=True)
            else:
                print("Trajet vide, retour au menu")
        elif choix == "4":
            plan_and_go_from_inputs(arduino_conn)
        elif choix == "Q":
            break
        else:
            print("Choix invalide!")

    disconnect_arduino()


if __name__ == "__main__":
    from menu_ui import launch_gui

    try:
        launch_gui()
    finally:
        disconnect_arduino()
