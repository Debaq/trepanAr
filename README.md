# TrepanAr

**Abrir el cráneo sin abrirlo.** Apoyas una sonda sobre un cráneo de plástico y en la pantalla
aparece el **cerebro real de una persona**, cortado justo bajo la punta: como una ecografía,
pero sobre el escáner HiP-CT de un cerebro completo donado a la ciencia.

TrepanAr nació como demo de museo y ahora es un proyecto propio: un instrumento de enseñanza de
neuroanatomía que une una sonda inercial de bajo costo (ESP32-C3 + MPU9250), imanes como faros
en el cráneo y el cerebro LADAF-2021-17 del [Human Organ Atlas](https://human-organ-atlas.esrf.fr)
con 95 regiones anatómicas con nombre en español.

> *Trepanar*: perforar el cráneo. La práctica quirúrgica más antigua que se conoce.
> Aquí se hace con rayos X de sincrotrón y un lápiz.

---

## Qué hace

- **Corte bajo la punta.** La orientación de la sonda define un corte oblicuo del volumen, centrado
  en el punto del cráneo donde está apoyada.
- **Profundidad con gestos.** Girar la sonda como un destornillador hunde o sube el corte
  (1 vuelta = 3 cm); un golpecito baja 5 mm, uno doble sube 5 mm.
- **Imanes como faros.** Imanes de neodimio en frente, Cz, temporales y occipital corrigen la deriva
  del giroscopio cada vez que la sonda pasa quieta por uno. El de la frente devuelve la vista a la
  superficie. El magnetómetro solo detecta imanes; nunca entra en el cálculo de la orientación.
- **Regiones con nombre.** 95 estructuras (segmentación FastSurfer) con colores y nombre bajo la
  mira o bajo el mouse, p. ej. *«Giro precentral: movimiento (izquierdo)»*.
- **Vistas 3D** del cerebro por lóbulos con el plano de corte y la sonda, en pantallas anchas.
- **Calibración guiada** por voz y dibujos: no supone nada sobre cómo va montado el sensor dentro
  de la sonda; aprende ejes, polaridad de cada imán y dirección de los golpes.
- **Funciona sin sonda**: con el mouse se recorre la cabeza y con la rueda se cambia la profundidad.

## Hardware

| pieza | detalle |
|---|---|
| Microcontrolador | ESP32-C3 Super Mini (USB CDC) |
| IMU | MPU9250 (acelerómetro, giroscopio, magnetómetro) en la punta de la sonda |
| Cráneo | cráneo anatómico de plástico |
| Imanes | neodimio en frente, Cz (polaridad invertida), temporal izq., temporal der., occipital |
| PC | Linux x86-64 con escritorio (X11 o Wayland) |

Firmware en [`firmware/hoa_imu/`](firmware/hoa_imu/) (Arduino). Para compilar y subir:
`firmware/flashear.sh` (arduino-cli); para grabar un binario ya compilado:
`firmware/flashear_lapiz.sh` (esptool).

## Instalación

Con [uv](https://docs.astral.sh/uv/) (Python 3.11–3.13):

```bash
git clone https://github.com/Debaq/trepanAr.git
cd trepanAr
uv run trepanar.py --paquetes      # la primera vez: elegir y descargar el cerebro
```

O con pip: `pip install .` deja el comando `trepanar`.

**Puerto serie:** el usuario debe estar en el grupo `dialout` (Debian/Ubuntu) o `uucp` (Arch).
Sin él funciona igual, pero sin sonda.

### Datos

Los cerebros **no van en el repositorio**. La primera vez se abre la pantalla de paquetes, que
los descarga desde la fuente oficial (CC BY 4.0):

| paquete | contenido | descarga | en disco |
|---|---|---|---|
| `ladaf17-regiones` (recomendado) | imagen 247 µm + 95 regiones | ~330 MB | 275 MB |
| `ladaf17-regiones-liviano` | imagen 494 µm + 95 regiones | ~45 MB | 83 MB |

En desarrollo se guardan en `datos/`; el binario usa `~/.local/share/trepanar/`
(si existe la carpeta del nombre anterior, `hoa-imu-demo`, se reutiliza).

## Uso

```bash
uv run trepanar.py                        # sonda en /dev/ttyACM*, se reconecta sola
uv run trepanar.py --completa --monitor 1 # pantalla completa en otro monitor
uv run trepanar.py --sin-imu              # solo mouse y teclado
uv run trepanar.py --instalar ladaf17-regiones   # preparar un equipo sin ventana
```

| tecla | acción |
|---|---|
| `p` | calibrar la sonda (guía en pantalla) |
| `m` | configurar el montaje del sensor dentro de la sonda |
| `espacio` | armar la vuelta a la superficie (se completa en el imán de la frente) |
| `e` | mostrar u ocultar regiones |
| `b` | ventana de imanes (distancia y polaridad) |
| `F2` | pantalla de paquetes |
| `Esc` / `q` | salir |

Binario autónomo para Linux: `./construir_binario.sh` → `dist/trepanar-linux.tar.gz`.

## Documentación

- [GUIA-VISITANTES.md](GUIA-VISITANTES.md): texto para el público (qué se ve, desafíos, datos curiosos).
- [LEEME-TECNICO.md](LEEME-TECNICO.md): montaje, calibración, gestos, imanes y mantenimiento.

## Estructura

| archivo | qué hace |
|---|---|
| `trepanar.py` | programa principal: cortes, regiones, sonda, gestos, pantalla |
| `paquetes.py` | catálogo de cerebros, descarga, conversión, nombres de regiones en español |
| `instalador.py` | pantalla de paquetes |
| `vista3d.py` | mini vista 3D (lóbulos, plano de corte y sonda) |
| `hoa_bajar.py` | descarga directa de volúmenes del Human Organ Atlas |
| `grabar_guiado.py`, `analizar_*.py` | grabaciones guiadas de la sonda y su análisis |
| `firmware/` | firmware Arduino de la sonda y scripts para grabarlo |

## Créditos y datos

**El cerebro y sus regiones no son nuestros.** Son datos abiertos, licencia CC BY 4.0:

- **Imágenes:** Human Organ Atlas, cerebro completo del donante LADAF-2021-17, escaneado en el
  ESRF (línea BM18) con HiP-CT. DOI [10.15151/ESRF-DC-2268032801](https://doi.org/10.15151/ESRF-DC-2268032801).
- **Regiones:** segmentación FastSurfer de Chourrout et al., *The Extremely Brilliant Brain: An
  Isotropic Microscale Human Brain Dataset*
  ([artículo](https://pmc.ncbi.nlm.nih.gov/articles/PMC12874060/),
  [repositorio](https://github.com/UCL-MXI-Bio/2025-chourrout-extremely-brilliant-brain)).

Agradecimiento a la persona donante y a su familia, y a los equipos del ESRF, University College
London y el Laboratorio de Anatomía de los Alpes Franceses (LADAF).

**Autores:** Nicolás Baier-Quezada, Vanessa Uribe-Hernández, Fernanda López-Moncada y
Benjamín Catipillán-Ulloa — TecMedHub · Universidad Austral de Chile, Sede Puerto Montt.

## Licencia

Código: por definir. Datos: CC BY 4.0 (Human Organ Atlas), se descargan aparte y no se redistribuyen aquí.
