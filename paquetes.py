"""Paquetes de datos que TrepanAr descarga e instala.

Los cerebros NO son nuestros: son del Human Organ Atlas (ESRF / UCL), con licencia
CC BY 4.0. TrepanAr los descarga de la fuente oficial la primera vez y los guarda en
~/.local/share/trepanar/paquetes/<id>/ (o datos/paquetes/ en desarrollo):

  vol.npy    imagen uint8 en orden (z, y, x)
  seg.npy    regiones (uint16, etiquetas FreeSurfer) en orden (z, y, x), opcional
  meta.json  resolución, orientación anatómica, etiquetas y créditos
"""
import json
import shutil
import threading
from pathlib import Path

import numpy as np

EBB = "https://storage.googleapis.com/ucl-hip-ct-ebb-hseid5pvpp8vgtzt"
IMAGEN_LADAF17 = f"{EBB}/sub-01/ses-01/micr/sub-01_ses-01_sample-brain_XPCT.ome.zarr"
SEG_LADAF17 = (f"{EBB}/derivatives/sub-01/ses-01/micr/"
               "sub-01_ses-01_sample-brain_seg-freesurfer_res-494um_desc-FastSurfer_dseg.precomputed")

CREDITOS_LADAF17 = {
    "fuente": "Human Organ Atlas — ESRF / University College London",
    "donante": "LADAF-2021-17 (hombre, 63 años), Laboratorio de Anatomía de los Alpes Franceses",
    "licencia": "CC BY 4.0",
    "cita": ("Chourrout et al., 'The Extremely Brilliant Brain: An Isotropic Microscale Human "
             "Brain Dataset' (2026). Datos: doi:10.15151/ESRF-DC-2268032801. "
             "Regiones: segmentación FastSurfer de los autores."),
    "web": "https://human-organ-atlas.esrf.fr",
}

CATALOGO = [
    {
        "id": "ladaf17-regiones",
        "nombre": "Cerebro completo con regiones",
        "descripcion": ("Cerebro humano completo escaneado con HiP-CT en el sincrotrón ESRF, "
                        "con 95 regiones anatómicas con nombre. Detalle de 0,25 mm."),
        "tamano_mb": 330,
        "recomendado": True,
        "imagen": {"zarr": IMAGEN_LADAF17, "nivel": 5},
        "segmentacion": {"precomputed": SEG_LADAF17, "factor": 2},
        "creditos": CREDITOS_LADAF17,
    },
    {
        "id": "ladaf17-regiones-liviano",
        "nombre": "Cerebro completo con regiones (liviano)",
        "descripcion": ("El mismo cerebro y las mismas regiones con la mitad de detalle "
                        "(0,5 mm). Para computadores modestos o conexiones lentas."),
        "tamano_mb": 45,
        "recomendado": False,
        "imagen": {"zarr": IMAGEN_LADAF17, "nivel": 6},
        "segmentacion": {"precomputed": SEG_LADAF17, "factor": 1},
        "creditos": CREDITOS_LADAF17,
    },
]

# ---------- nombres en español (FreeSurfer / atlas DKT) ----------
SUBCORTICALES = {
    "Cerebral-White-Matter": "Sustancia blanca cerebral",
    "Lateral-Ventricle": "Ventrículo lateral",
    "Inf-Lat-Vent": "Asta temporal del ventrículo lateral",
    "Cerebellum-White-Matter": "Sustancia blanca del cerebelo",
    "Cerebellum-Cortex": "Corteza del cerebelo",
    "Thalamus": "Tálamo",
    "Thalamus-Proper": "Tálamo",
    "Caudate": "Núcleo caudado",
    "Putamen": "Putamen",
    "Pallidum": "Globo pálido",
    "Hippocampus": "Hipocampo",
    "Amygdala": "Amígdala",
    "Accumbens-area": "Núcleo accumbens",
    "VentralDC": "Diencéfalo ventral",
    "choroid-plexus": "Plexo coroideo",
    "vessel": "Vaso sanguíneo",
    "3rd-Ventricle": "Tercer ventrículo",
    "4th-Ventricle": "Cuarto ventrículo",
    "5th-Ventricle": "Quinto ventrículo",
    "Brain-Stem": "Tronco encefálico",
    "CSF": "Líquido cefalorraquídeo",
    "WM-hypointensities": "Sustancia blanca",
    "Optic-Chiasm": "Quiasma óptico",
    "CC_Posterior": "Cuerpo calloso", "CC_Mid_Posterior": "Cuerpo calloso", "CC_Central": "Cuerpo calloso",
    "CC_Mid_Anterior": "Cuerpo calloso", "CC_Anterior": "Cuerpo calloso",
}
# (nombre, lóbulo)
CORTICALES = {
    "caudalanteriorcingulate": ("Cíngulo anterior", "límbico"),
    "caudalmiddlefrontal": ("Giro frontal medio", "frontal"),
    "cuneus": ("Cuña", "occipital"),
    "entorhinal": ("Corteza entorrinal", "temporal"),
    "fusiform": ("Giro fusiforme", "temporal"),
    "inferiorparietal": ("Lobulillo parietal inferior", "parietal"),
    "inferiortemporal": ("Giro temporal inferior", "temporal"),
    "isthmuscingulate": ("Istmo del cíngulo", "límbico"),
    "lateraloccipital": ("Corteza occipital lateral", "occipital"),
    "lateralorbitofrontal": ("Corteza orbitofrontal lateral", "frontal"),
    "lingual": ("Giro lingual", "occipital"),
    "medialorbitofrontal": ("Corteza orbitofrontal medial", "frontal"),
    "middletemporal": ("Giro temporal medio", "temporal"),
    "parahippocampal": ("Giro parahipocampal", "temporal"),
    "paracentral": ("Lobulillo paracentral", "frontal y parietal"),
    "parsopercularis": ("Pars opercularis", "frontal"),
    "parsorbitalis": ("Pars orbitalis", "frontal"),
    "parstriangularis": ("Pars triangularis", "frontal"),
    "pericalcarine": ("Corteza visual primaria", "occipital"),
    "postcentral": ("Giro poscentral: tacto", "parietal"),
    "posteriorcingulate": ("Cíngulo posterior", "límbico"),
    "precentral": ("Giro precentral: movimiento", "frontal"),
    "precuneus": ("Precúneo", "parietal"),
    "rostralanteriorcingulate": ("Cíngulo anterior", "límbico"),
    "rostralmiddlefrontal": ("Giro frontal medio", "frontal"),
    "superiorfrontal": ("Giro frontal superior", "frontal"),
    "superiorparietal": ("Lobulillo parietal superior", "parietal"),
    "superiortemporal": ("Giro temporal superior: audición", "temporal"),
    "supramarginal": ("Giro supramarginal", "parietal"),
    "transversetemporal": ("Corteza auditiva primaria", "temporal"),
    "insula": ("Ínsula", "insular"),
    "frontalpole": ("Polo frontal", "frontal"),
    "temporalpole": ("Polo temporal", "temporal"),
    "bankssts": ("Surco temporal superior", "temporal"),
}


def nombre_es(nombre_en):
    """'ctx-lh-precentral' -> ('Giro precentral: movimiento (izquierdo)', 'lóbulo frontal')."""
    if nombre_en.startswith(("ctx-lh-", "ctx-rh-")):
        lado = "izquierdo" if nombre_en[4] == "l" else "derecho"
        clave = nombre_en[7:]
        nombre, lobulo = CORTICALES.get(clave, (clave, ""))
        if lado == "izquierdo" and clave in ("parsopercularis", "parstriangularis"):
            nombre += " (área de Broca)"
        return f"{nombre} ({lado})", (f"lóbulo {lobulo}" if lobulo else "corteza cerebral")
    lado = ""
    for pre, txt in (("Left-", "izquierdo"), ("Right-", "derecho")):
        if nombre_en.startswith(pre):
            lado, nombre_en = txt, nombre_en[len(pre):]
    nombre = SUBCORTICALES.get(nombre_en, nombre_en.replace("-", " "))
    return (f"{nombre} ({lado})" if lado else nombre), "estructura profunda"


# ---------- instalación ----------
def carpeta(base, pid):
    return Path(base) / "paquetes" / pid


def instalado(base, pid):
    c = carpeta(base, pid)
    return (c / "vol.npy").exists() and (c / "meta.json").exists()


def borrar(base, pid):
    shutil.rmtree(carpeta(base, pid), ignore_errors=True)


def por_id(pid):
    return next((p for p in CATALOGO if p["id"] == pid), None)


class Cancelado(Exception):
    pass


class Descarga(threading.Thread):
    """Descarga e instala un paquete en segundo plano. Lee .progreso (0-1), .estado y .error."""

    def __init__(self, base, paquete):
        super().__init__(daemon=True)
        self.base, self.p = Path(base), paquete
        self.progreso, self.estado, self.error, self.listo = 0.0, "preparando…", None, False
        self.cancelar = threading.Event()

    def run(self):
        destino = carpeta(self.base, self.p["id"])
        temporal = destino.with_name(destino.name + ".descargando")
        try:
            shutil.rmtree(temporal, ignore_errors=True)
            temporal.mkdir(parents=True)
            self._instalar(temporal)
            shutil.rmtree(destino, ignore_errors=True)
            temporal.rename(destino)
            self.estado, self.progreso, self.listo = "instalado", 1.0, True
        except Cancelado:
            self.estado = "cancelado"
        except Exception as e:  # red caída, servidor, disco lleno…
            self.error = f"{type(e).__name__}: {e}"
            self.estado = "error"
        finally:
            shutil.rmtree(temporal, ignore_errors=True)

    def _avance(self, frac, estado):
        if self.cancelar.is_set():
            raise Cancelado()
        self.progreso, self.estado = frac, estado

    def _instalar(self, dest):
        seg, etiquetas = None, {}
        if self.p.get("segmentacion"):
            seg, etiquetas = self._bajar_segmentacion(self.p["segmentacion"]["precomputed"], 0.0, 0.1)
        vol, voxel_um = self._bajar_imagen(self.p["imagen"], seg, self.p.get("segmentacion"), dest, 0.1, 0.95)
        self._avance(0.96, "calculando orientación…")
        meta = {"id": self.p["id"], "nombre": self.p["nombre"], "voxel_um": voxel_um,
                "creditos": self.p["creditos"], "factor_seg": 1}
        if seg is not None:
            meta["factor_seg"] = self.p["segmentacion"]["factor"]
            meta["anatomia"] = anatomia_desde_segmentacion(seg)  # seg en (z, y, x)
            meta["etiquetas"] = etiquetas
            np.save(dest / "seg.npy", seg)
        (dest / "meta.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2))

    def _bajar_segmentacion(self, url, f0, f1):
        """Neuroglancer precomputed con compresión 'compresso' (bloques de 64³)."""
        import itertools

        import compresso
        import fsspec
        fs = fsspec.filesystem("http")
        info = json.loads(fs.cat(f"{url}/info"))
        escala = info["scales"][0]
        tam, (c, _, _) = escala["size"], escala["chunk_sizes"][0]
        props = json.loads(fs.cat(f"{url}/{info['segment_properties']}/info"))["inline"]
        nombres = next(p["values"] for p in props["properties"] if p["id"] == "label")
        colores = next((p["values"] for p in props["properties"] if p["id"] == "segmentColors"), None)
        etiquetas = {}
        for i, (sid, nom) in enumerate(zip(props["ids"], nombres)):
            es, lobulo = nombre_es(nom)
            etiquetas[sid] = {"en": nom, "es": es, "zona": lobulo,
                              "color": colores[i] if colores else "#ffffff"}
        bloques = []
        for ijk in itertools.product(*[range((s + c - 1) // c) for s in tam]):
            lo = [i * c for i in ijk]
            hi = [min(l + c, s) for l, s in zip(lo, tam)]
            bloques.append((lo, hi, f"{url}/{escala['key']}/{lo[0]}-{hi[0]}_{lo[1]}-{hi[1]}_{lo[2]}-{hi[2]}"))
        seg = np.zeros(tam, np.uint16)  # (x, y, z)
        for k in range(0, len(bloques), 25):
            lote = bloques[k:k + 25]
            datos = fs.cat([u for *_, u in lote], on_error="return")  # bloque inexistente = fondo
            for lo, hi, u in lote:
                d = datos.get(u)
                if isinstance(d, bytes) and d.startswith(b"cpso"):
                    a = np.asarray(compresso.decompress(d))
                    seg[lo[0]:hi[0], lo[1]:hi[1], lo[2]:hi[2]] = a[..., 0] if a.ndim == 4 else a
            self._avance(f0 + (f1 - f0) * min(1, (k + 25) / len(bloques)), "bajando regiones…")
        return np.ascontiguousarray(seg.transpose(2, 1, 0)), etiquetas  # -> (z, y, x)

    def _bajar_imagen(self, img, seg, info_seg, dest, f0, f1):
        """OME-Zarr por tajadas (para mostrar progreso) -> uint8 con ventana de contraste."""
        import zarr
        grupo = zarr.open_group(img["zarr"], mode="r")
        attrs = grupo.attrs
        ms = (attrs["ome"] if "ome" in attrs else attrs)["multiscales"][0]
        ds = ms["datasets"][img["nivel"]]
        ejes = [e["name"] if isinstance(e, dict) else e for e in ms.get("axes", ["x", "y", "z"])]
        voxel_um = float(ds["coordinateTransformations"][0]["scale"][0])
        arr = grupo[ds["path"]]
        orden = [ejes.index(e) for e in "zyx"]
        forma = [arr.shape[i] for i in orden]
        # temporal uint16 en disco (no ocupa RAM), luego ventana -> uint8
        crudo = np.lib.format.open_memmap(dest / "crudo.npy", mode="w+", dtype=arr.dtype, shape=tuple(forma))
        ez = ejes.index("z")
        paso = arr.chunks[ez] if hasattr(arr, "chunks") else 64
        for z0 in range(0, forma[0], paso):
            sel = [slice(None)] * 3
            sel[ez] = slice(z0, min(z0 + paso, forma[0]))
            crudo[z0:z0 + paso] = np.asarray(arr[tuple(sel)]).transpose(orden)
            self._avance(f0 + (f1 - f0) * 0.9 * min(1, (z0 + paso) / forma[0]), "bajando imagen…")
        self._avance(f0 + (f1 - f0) * 0.92, "ajustando contraste…")
        muestra = crudo[::2, ::2, ::2]
        if seg is not None:  # contraste medido dentro del cerebro
            fac = info_seg["factor"]
            zz, yy, xx = [np.minimum(np.arange(0, n, 2) // fac, s - 1) for n, s in zip(forma, seg.shape)]
            dentro = seg[np.ix_(zz, yy, xx)] > 0
            valores = muestra[dentro] if dentro.any() else muestra[muestra > 0]
        else:
            valores = muestra[muestra > 0]
        lo, hi = np.percentile(valores, [0.5, 99.5])
        vol = np.lib.format.open_memmap(dest / "vol.npy", mode="w+", dtype=np.uint8, shape=tuple(forma))
        for z0 in range(0, forma[0], 32):
            t = (crudo[z0:z0 + 32].astype(np.float32) - lo) / max(hi - lo, 1)
            vol[z0:z0 + 32] = (np.clip(t, 0, 1) * 255).astype(np.uint8)
        vol.flush()
        del vol, crudo
        (dest / "crudo.npy").unlink()
        return None, voxel_um


def anatomia_desde_segmentacion(seg):
    """Ejes anatómicos en coordenadas (x, y, z) del volumen, calculados con las etiquetas.
    quiralidad = -1 si los ejes del volumen están en espejo respecto de la anatomía."""
    zyx = {}

    def centro(ids):
        clave = tuple(ids)
        if clave not in zyx:
            zyx[clave] = np.argwhere(np.isin(seg, ids)).mean(0)[::-1]  # -> (x, y, z)
        return zyx[clave]

    izq = centro([2, 1028, 1024]) - centro([41, 2028, 2024])
    ant = centro([1028, 2028, 1027, 2027]) - centro([1011, 2011, 8, 47])
    sup = centro([1017, 2017, 1029, 2029]) - centro([16, 8, 47])
    a = ant / np.linalg.norm(ant)
    s = sup - (sup @ a) * a
    s /= np.linalg.norm(s)
    l = izq - (izq @ a) * a - (izq @ s) * s
    l /= np.linalg.norm(l)
    quiral = 1.0 if np.cross(a, s) @ (-l) > 0 else -1.0  # físico: anterior × superior = derecha
    return {"anterior": a.tolist(), "superior": s.tolist(), "izquierda": l.tolist(), "quiralidad": quiral}
