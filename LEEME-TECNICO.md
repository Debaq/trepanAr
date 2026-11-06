# TrepanAr — guía técnica

> Para visitantes y estudiantes: ver [GUIA-VISITANTES.md](GUIA-VISITANTES.md).

Muestra cortes del cerebro completo **LADAF-2021-17** (Human Organ Atlas, ESRF), con 95
regiones anatómicas con nombre, según dónde apoyes el lápiz (la sonda) sobre un cráneo de plástico.

**Los datos no van dentro del programa**: la primera vez se abre la **pantalla de paquetes**,
que los descarga desde la fuente oficial (CC BY 4.0) y los guarda en
`~/.local/share/trepanar/paquetes/`.

| paquete | contenido | descarga | en disco |
|---|---|---|---|
| `ladaf17-regiones` (recomendado) | imagen 247 µm + 95 regiones | ~330 MB | 275 MB |
| `ladaf17-regiones-liviano` | imagen 494 µm + 95 regiones | ~45 MB | 83 MB |

## Contenido

| archivo | qué es |
|---|---|
| `trepanar` | la demo (binario Linux x86-64 con todas las librerías; los cerebros se descargan aparte) |
| `firmware/hoa_imu_esp32c3.bin` | firmware del lápiz (ESP32-C3 Super Mini + MPU9250) |
| `firmware/flashear_lapiz.sh` | graba el firmware en un lápiz nuevo |
| `firmware/respaldo_firmware_original.bin` | respaldo del firmware que traía la placa |

## Requisitos del PC

- Linux x86-64 con escritorio (X11 o Wayland).
- Permiso para el puerto serie: el usuario debe estar en el grupo `uucp` (Arch) o `dialout`
  (Debian/Ubuntu). Una vez: `sudo usermod -aG dialout $USER` y volver a iniciar sesión.
  Sin ella la demo funciona igual, pero las órdenes salen solo en pantalla.

- Internet solo la primera vez (para descargar el paquete).

## Uso

```bash
./trepanar                 # pantalla completa en el monitor 0 (la 1ª vez: pantalla de paquetes)
./trepanar --monitor 1     # en otro monitor
./trepanar --no-completa   # en ventana
./trepanar --paquetes      # volver a la pantalla de paquetes (o F2 dentro de la demo)
./trepanar --instalar ladaf17-regiones   # descargar sin ventana (preparar equipos)
```

Teclas: `b` ventana de imanes (semáforo de distancia gris/verde/amarillo/rojo y polaridad Norte/Sur hacia la sonda; acercar a 1-2 cm sin tocar para que no sature), `m` configura la sonda (montaje del sensor), `espacio` arma la superficie (se completa con el lápiz quieto en el imán de la frente), `p` calibra el lápiz (guía en pantalla), `e` muestra u oculta las regiones (colores y nombre; parte oculto), `F2` paquetes.
Salir: `Esc` o `q`. El lápiz se detecta solo en `/dev/ttyACM*` y se reconecta si se desenchufa;
si la demo arranca sin lápiz, lo espera.

**Mouse:** pasar el puntero sobre el corte muestra el nombre de la estructura; clic = resaltarla. **Sin lápiz**, arrastrar con el botón apretado recorre la cabeza
(horizontal = alrededor, vertical = subir/bajar); rueda o flechas ↑↓ = profundidad.

**Pantallas anchas** (más anchas que 1,25 × su alto): el corte va al centro, a la izquierda dos
vistas 3D (de frente y de lado) y a la derecha el nombre, la profundidad y la ayuda.
Para probar sin ventana: `SDL_VIDEODRIVER=dummy HOA_PRUEBA="1920x1080;960,540;captura.png" python trepanar.py --completa`.

## Montaje

- Imanes de neodimio en el cráneo: **frente, Cz, temporal izquierdo, temporal derecho, occipital**
  (uno por punto; si un punto marca poco campo, dos apilados). Deben quedar justo bajo
  donde se apoya la punta del lápiz.
- El sensor va en la **punta** del lápiz, con su eje Z a lo largo del lápiz.

## Funcionamiento (sin teclado)

| acción con el lápiz | efecto |
|---|---|
| apoyarlo en el cráneo | el corte se ubica bajo la punta |
| girarlo sobre su eje (destornillador) | más/menos hondo (1 vuelta = 3 cm) |
| golpecito con el dedo (en la dirección calibrada) | 5 mm más hondo (se aplica ~0,5 s después, para distinguirlo del doble) |
| doble golpecito (dos en menos de 0,5 s) | 5 mm hacia la superficie |
| **espacio** (arma) y luego apoyar la sonda en el imán de la frente | se completa con el imán + 0,2 s quieta (< 20 °/s), o 1 s en el imán aunque tiemble. La vista queda en el polo rostral, en la superficie; corrige toda la orientación. En pantalla: «ARMADO · imán: sí/no · quieta: sí/no». Espacio otra vez cancela; se desarma sola a los 20 s. Cada intento queda en `~/.local/share/trepanar/registro_superficie.log` |
| tecla **m** | configurar la sonda: montaje del sensor (3 posiciones, guía en pantalla) |
| tecla **p** | calibrar (guía en pantalla) |

**Primera vez:** si no hay calibración, empieza sola (también con `p`). Durante la calibración la
pantalla muestra una **guía visual**: la cabeza de lado y desde arriba con el punto a tocar
parpadeando, el estado (escucha / ve al punto / quieto con barra / deslizando / golpes contados) y
los pasos hechos. No supone nada sobre cómo
va montado el sensor dentro del lápiz: (1) sonda **plana sobre Cz** (vértice), como una lupa — la gravedad da la
perpendicular a su cara (el eje que importa, sin importar hacia dónde quede el mango); (2) frente; (3) **deslizar de la frente a la nuca por encima** — define el plano
medio (adelante/atrás) y el eje izquierda-derecha; (4) nuca; (5-6) temporales. El eje se refina
con todos los puntos (error típico 3–8° aunque los toques no sean perfectos). Cada punto se
registra cuando terminó la orden hablada, el lápiz se movió desde el punto anterior y quedó quieto
~1,4 s (al final avisa en pantalla qué punto no calzó); ahí también mide cuánto marca cada imán y el campo
de la Tierra en el camino, para fijar un umbral por imán y avisar si alguno no se detecta) y luego
**golpear el lápiz 5 veces, como sea más
cómodo** (de lado, de frente o en la punta de arriba). Aprende la dirección del golpe y la fuerza
mínima, separándola de los apoyos en el cráneo grabados al tocar los puntos. Si golpeas en la
misma dirección en que apoyas el lápiz (en la punta de arriba), solo los separa la fuerza: si
quedan muy parecidos lo avisa ("golpea un poco más fuerte"). Si no detecta al menos 3 golpes,
usa el valor de fábrica (cualquier dirección, 2,5 g). Al final dice el error
del ajuste (menos de ~20° está bien). Se guarda en `~/.local/share/trepanar/`.

Si el lápiz se desorienta (o tras desenchufarlo): **espacio** y luego el lápiz quieto en el imán de
la frente. La vista queda en el polo rostral (corte casi coronal, mira en la cisura interhemisférica), en la
superficie, y la orientación queda corregida, sin recalibrar. Cualquier imán, con el lápiz quieto encima, corrige la orientación hacia su punto (deriva); solo
el de la frente completa la superficie. Además el firmware
vuelve a medir solo el sesgo del giroscopio cada vez que el lápiz queda quieto ~1 s. La frente se reconoce aunque el rumbo haya
derivado (por la inclinación del lápiz y por lo que marca su imán). Los otros imanes ya no hacen nada.

## Configurar la sonda (montaje del sensor)

La demo necesita saber cómo va el sensor dentro de la sonda: hacia dónde apunta la **cara
detectora** (la que se apoya en el cráneo, como una lupa) y el **mango**. Con una sonda nueva
(sin `montaje.json`) la configuración aparece sola; para repetirla, tecla **m**. Son 3 posiciones
con dibujo en pantalla, cada una medida al quedar quieta ~2 s:

1. sonda sobre la mesa con la **cara detectora hacia abajo**;
2. en la mano con el **mango hacia arriba** y la cara mirándote;
3. sobre la mesa con la **cara detectora hacia arriba**.

Al final revisa la coherencia (cara abajo vs arriba < 20°, cara–mango 60–120°), guarda
`~/.local/share/trepanar/montaje.json` y lo aplica. Después: calibrar con **p**.

## Lápiz nuevo

```bash
cd firmware
./flashear_lapiz.sh            # usa esptool (pipx install esptool)
```

Luego recalibrar con el gesto de 8 s en la frente (o borrar
`~/.local/share/trepanar/calib_puntos.json` para que calibre al abrir).

## Datos

Human Organ Atlas — LADAF-2021-17, cerebro completo, 20.176 µm, BM18, ESRF
(DOI y licencia en https://human-organ-atlas.esrf.fr). Versión reducida a 323 µm.

## Desde el código fuente (cualquier sistema con Python 3.11–3.13)

```bash
pip install -r requirements.txt        # o: pip install .   (deja el comando trepanar)
python trepanar.py --paquetes
```

Con [uv](https://docs.astral.sh/uv/) no hace falta instalar nada a mano: `uv run trepanar.py`.

Para regenerar el binario y el paquete de distribución: `./construir_binario.sh`
(deja `dist/trepanar-linux.tar.gz`).

| archivo | qué hace |
|---|---|
| `trepanar.py` | la demo (cortes, regiones, sonda, gestos, pantalla) |
| `paquetes.py` | catálogo de cerebros, descarga, conversión, nombres de regiones en español |
| `instalador.py` | pantalla de paquetes |
| `vista3d.py` | mini vista 3D (cerebro por lóbulos, corte y sonda) |
| `firmware/hoa_imu/` | firmware del lápiz (Arduino, ESP32-C3) |
| `grabar_guiado.py`, `analizar_*.py` | grabaciones guiadas por voz y su análisis |

El magnetómetro **nunca** entra en el cálculo de la orientación (los imanes del cráneo la desviarían): solo sirve para detectar imanes.

**Polaridad de los imanes:** el imán de **Cz va al revés** que los demás. La calibración **aprende**
qué polaridad marca cada imán en esta sonda (al acercarse, antes de saturar), sin depender de la
convención Norte/Sur. En uso, si la polaridad leída corresponde a un solo punto (Cz), se reconoce
ese punto aunque la orientación esté corrida, y ese imán no completa la superficie. Con
calibraciones anteriores a esto no se usa la polaridad.
