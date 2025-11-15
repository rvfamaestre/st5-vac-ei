from __future__ import annotations
from dataclasses import dataclass
from typing import Optional, Tuple
import cv2
import numpy as np

# ----------------- Paramètres (simples à régler) -----------------
ROI_START_RATIO = 0.00  # bande basse (0% -> 100% de la hauteur)
ROI_END_RATIO = 1.00    # CRITIQUE: Doit être 1.00 pour couvrir toute l'image !

# Seuils pour CROIX classique (verticale + horizontale)
V_MIN = 0.20            # densité verticale minimale (0..1)
H_MIN = 0.15            # densité horizontale minimale (0..1)
MIN_HORIZ_WIDTH = 0.05  # largeur minimale (RÉDUIT: 0.10 -> 0.05 pour intersections partielles)

# Seuils pour T (principalement horizontale)
H_MIN_T = 0.20          # densité horizontale pour T (RÉDUIT: 0.25 -> 0.20)
WIDTH_MIN_T = 0.08      # largeur minimale pour T (RÉDUIT: 0.20 -> 0.08 pour T partielles)
V_MAX_T = 0.35          # densité verticale max pour T pur (nouveau)

# Seuils pour BORDURE (ligne horizontale simple)
H_MIN_BORDER = 0.20     # densité horizontale pour bordure
WIDTH_MIN_BORDER = 0.30 # largeur minimale pour bordure (large)
V_MAX_BORDER = 0.15     # densité verticale max pour bordure (très faible)

EXCLUDE_VERT_FRAC = 0.02  # largeur du couloir exclu autour de la colonne verticale
WHITE_RATIO_MIN = 0.02    # ratio de blanc minimum dans la ROI (RÉDUIT: 0.05 -> 0.03)

# ----------------- Dataclass résultat -----------------
@dataclass(frozen=True)
class IntersectionDetectionResult:
    coordinates: Optional[Tuple[int, int]]
    white_ratio: float
    vertical_density: float
    horizontal_density: float
    area_ratio: float
    width_ratio: float
    height_ratio: float

# ----------------- Utilitaires -----------------
def _build_mask_adaptive(image_bgr: np.ndarray) -> np.ndarray:
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Seuil adaptatif -> 255 pour "clair"
    bin_img = cv2.adaptiveThreshold(
        gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
        cv2.THRESH_BINARY, 11, -5
    )
    
    # Nettoyage léger
    bin_img = cv2.morphologyEx(bin_img, cv2.MORPH_OPEN, np.ones((3,3), np.uint8), iterations=1)
    bin_img = cv2.morphologyEx(bin_img, cv2.MORPH_CLOSE, np.ones((5,5), np.uint8), iterations=1)
    
    return bin_img

def _longest_run_length(row: np.ndarray) -> int:
    r = (row > 0).astype(np.uint8)
    if r.size == 0:
        return 0
    
    # Trouver transitions
    diff = np.diff(np.concatenate(([0], r, [0])))
    starts = np.where(diff == 1)[0]
    ends = np.where(diff == -1)[0]
    
    if starts.size == 0:
        return 0
    
    return int(np.max(ends - starts))

# ----------------- Détection -----------------
def detect_intersection(
    image: Optional[np.ndarray],
    feedback: bool = False,
    roi_start_ratio: float = ROI_START_RATIO,
) -> Tuple[Optional[int], Optional[int]]:
    res = detect_intersection_with_metrics(image, feedback, roi_start_ratio)
    return res.coordinates if res.coordinates is not None else (None, None)

def detect_intersection_with_metrics(
    image: Optional[np.ndarray],
    feedback: bool = False,
    roi_start_ratio: float = ROI_START_RATIO,
) -> IntersectionDetectionResult:
    
    if image is None:
        return IntersectionDetectionResult(None, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    
    H, W = image.shape[:2]
    y0 = max(0, min(H-1, int(H * roi_start_ratio)))
    y1 = max(y0+1, min(H, int(H * ROI_END_RATIO)))
    roi = image[y0:y1, :]
    
    mask = _build_mask_adaptive(roi)
    Hroi, Wroi = mask.shape[:2]
    
    # Protection contre ROI vide
    if Hroi == 0 or Wroi == 0:
        return IntersectionDetectionResult(None, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    
    white_ratio = float(mask.mean() / 255.0)
    
    # Protection contre masque complètement noir
    if white_ratio < 0.01:
        if feedback:
            print(f"[Intersection] Rejet: masque quasi-vide (white_ratio={white_ratio:.3f})")
        return IntersectionDetectionResult(None, white_ratio, 0.0, 0.0, 0.0, 0.0, 0.0)
    
    # Filtre sur le ratio de blanc minimum
    if white_ratio < WHITE_RATIO_MIN:
        if feedback:
            print(f"[Intersection] Rejet: pas assez de blanc (white_ratio={white_ratio:.3f} < {WHITE_RATIO_MIN})")
        return IntersectionDetectionResult(None, white_ratio, 0.0, 0.0, 0.0, 0.0, 0.0)
    
    # 1) Bande verticale dominante (somme par colonnes)
    col_sum = mask.sum(axis=0) / 255.0
    cmax = int(np.argmax(col_sum))
    v_density = float(col_sum[cmax] / Hroi)
    
    # 2) Barre horizontale : trouver la ligne la plus dense
    row_sum = mask.sum(axis=1) / 255.0
    rmax = int(np.argmax(row_sum))
    
    # 2a) Largeur COMPLÈTE (avant exclusion)
    run_len_full = _longest_run_length(mask[rmax, :])
    width_ratio_full = float(run_len_full / Wroi)
    
    # 2b) Densité horizontale APRÈS exclusion du couloir vertical
    excl = max(2, int(EXCLUDE_VERT_FRAC * Wroi))
    cross_mask = mask.copy()
    x0_ex = max(0, cmax - excl)
    x1_ex = min(Wroi, cmax + excl + 1)
    cross_mask[:, x0_ex:x1_ex] = 0
    
    row_sum_excl = cross_mask.sum(axis=1) / 255.0
    rmax_excl = int(np.argmax(row_sum_excl))
    h_density = float(row_sum_excl[rmax_excl] / Wroi)
    
    # 2c) NOUVEAU: Densité horizontale SANS exclusion (pour détecter T pur)
    h_density_no_excl = float(row_sum[rmax] / Wroi)
    
    # 3) Décision : 4 cas possibles
    
    # CAS 1: CROIX classique (verticale forte + horizontale forte)
    is_cross = (
        v_density >= V_MIN and 
        h_density >= H_MIN and 
        width_ratio_full >= MIN_HORIZ_WIDTH
    )
    
    # CAS 2: T pur (horizontale forte + verticale faible/absente)
    # On utilise h_density_no_excl pour ne pas pénaliser l'absence de verticale
    is_t_pure = (
        v_density < V_MAX_T and  # PAS de verticale forte
        h_density_no_excl >= H_MIN_T and  # horizontale forte (sans exclusion)
        width_ratio_full >= WIDTH_MIN_T and
        white_ratio >= WHITE_RATIO_MIN
    )
    
    # CAS 3: T avec petite verticale (compromis)
    is_t_mixed = (
        v_density < V_MIN and  # verticale présente mais insuffisante pour croix
        h_density_no_excl >= H_MIN_T and
        width_ratio_full >= WIDTH_MIN_T and
        white_ratio >= WHITE_RATIO_MIN * 1.5  # exiger un peu plus de blanc
    )
    
    # CAS 4: BORDURE (ligne horizontale simple, sans verticale)
    is_border = (
        v_density < V_MAX_BORDER and  # AUCUNE verticale significative
        h_density_no_excl >= H_MIN_BORDER and  # horizontale présente
        width_ratio_full >= WIDTH_MIN_BORDER and  # ligne large
        white_ratio >= WHITE_RATIO_MIN  # seuil minimal de blanc
    )
    
    if is_cross or is_t_pure or is_t_mixed or is_border:
        # Position de l'intersection = croisement (colonne cmax, ligne rmax)
        cx, cy = cmax, y0 + rmax
        
        # Métriques de forme
        height_ratio = 1.0 * (1) / Hroi
        area_ratio = white_ratio
        
        if feedback:
            if is_cross:
                detection_type = "CROIX"
            elif is_t_pure:
                detection_type = "T PUR"
            elif is_t_mixed:
                detection_type = "T MIXTE"
            else:
                detection_type = "BORDURE"
            
            print(f"[Intersection] ✓ {detection_type} @ ({cx},{cy})")
            print(f"  white={white_ratio:.3f} | v={v_density:.2f} | h={h_density:.2f} | h_no_excl={h_density_no_excl:.2f}")
            print(f"  width={width_ratio_full:.2f} | excl={excl}px")
        
        return IntersectionDetectionResult(
            (cx, cy),
            white_ratio,
            v_density,
            h_density_no_excl,  # On retourne la densité sans exclusion
            area_ratio,
            width_ratio_full,
            height_ratio
        )
    
    # Rejet : rien de suffisant
    if feedback:
        print(f"[Intersection] ✗ Rejet:")
        print(f"  v={v_density:.2f} (CROIX>={V_MIN}, T<{V_MAX_T}, BORDER<{V_MAX_BORDER})")
        print(f"  h_no_excl={h_density_no_excl:.2f} (CROIX>={H_MIN}, T>={H_MIN_T}, BORDER>={H_MIN_BORDER})")
        print(f"  width={width_ratio_full:.2f} (CROIX>={MIN_HORIZ_WIDTH}, T>={WIDTH_MIN_T}, BORDER>={WIDTH_MIN_BORDER})")
        print(f"  white={white_ratio:.3f} (min={WHITE_RATIO_MIN})")
    
    return IntersectionDetectionResult(None, white_ratio, v_density, h_density, 0.0, 0.0, 0.0)