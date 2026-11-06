"""Pantalla inicial: elegir, descargar y borrar paquetes de datos (pygame).

Se maneja con el mouse o con el teclado: flechas para elegir, Enter usar o descargar,
Supr borrar, Esc salir.
"""
import json
import textwrap

import paquetes

FONDO = (14, 17, 24)
TARJETA = (28, 33, 45)
TARJETA_SEL = (38, 46, 64)
TEXTO = (235, 238, 245)
TENUE = (150, 158, 175)
ACENTO = (255, 196, 40)
VERDE = (90, 200, 120)
ROJO = (230, 90, 90)
AZUL = (80, 150, 255)


class Boton:
    def __init__(self, rect, texto, color, accion):
        self.rect, self.texto, self.color, self.accion = rect, texto, color, accion


class PantallaPaquetes:
    def __init__(self, pantalla, base, config):
        self.pt, self.pg = pantalla, pantalla.pg
        self.base, self.config = base, config
        self.sel = 0
        self.descargas = {}  # id -> paquetes.Descarga
        self.confirmar_borrar = None
        self.mensaje = {}  # id -> texto de error/aviso
        self.botones = []

    # ---------- dibujo ----------
    def _f(self, tam, negrita=False):
        f = self.pt._fuente(int(tam * self.esc))
        f.set_bold(negrita)
        return f

    def _texto(self, texto, x, y, tam, color=TEXTO, negrita=False):
        s = self._f(tam, negrita).render(texto, True, color)
        self.pt.sup.blit(s, (x, y))
        return s.get_width(), s.get_height()

    def _boton(self, derecha, y, texto, color, accion):
        """Botón alineado a la derecha en x = derecha."""
        f = self._f(22, True)
        w, h = f.size(texto)
        ancho = w + 36 * self.esc
        r = self.pg.Rect(derecha - ancho, y, ancho, h + 18 * self.esc)
        encima = r.collidepoint(self.pg.mouse.get_pos())
        self.pg.draw.rect(self.pt.sup, tuple(min(255, c + 30) for c in color) if encima else color, r,
                          border_radius=int(10 * self.esc))
        self.pt.sup.blit(f.render(texto, True, (10, 12, 18)), (r.x + 18 * self.esc, r.y + 9 * self.esc))
        self.botones.append(Boton(r, texto, color, accion))
        return r

    def dibujar(self):
        sup, pg = self.pt.sup, self.pg
        W, H = self.pt.ancho, self.pt.alto
        self.esc = min(W / 1280, H / 800)
        e = self.esc
        self.botones = []
        sup.fill(FONDO)
        ancho = min(W - 80 * e, 1100 * e)
        x0 = (W - ancho) / 2
        y = 40 * e
        self._texto("Explora un cerebro real", x0, y, 52, TEXTO, True)
        y += 62 * e
        self._texto("Elige qué cerebro usar. Se descarga una sola vez desde el Human Organ Atlas.",
                    x0, y, 24, TENUE)
        y += 52 * e
        for i, p in enumerate(paquetes.CATALOGO):
            y = self._tarjeta(i, p, x0, y, ancho) + 18 * e
        # créditos
        c = paquetes.CATALOGO[self.sel]["creditos"]
        pie = [f"Datos: {c['fuente']} · licencia {c['licencia']} · {c['web']}",
               "Los cerebros no son de este proyecto: se descargan desde su fuente oficial y se usan citándola.",
               "Mouse o teclado: flechas para elegir · Enter usar o descargar · Supr borrar · Esc salir"]
        for k, t in enumerate(pie):
            self._texto(t, x0, H - (100 - 28 * k) * e, 20, TENUE)
        pg.display.flip()

    def _tarjeta(self, i, p, x, y, ancho):
        e, pg = self.esc, self.pg
        pid = p["id"]
        lineas = textwrap.wrap(p["descripcion"], 95)
        alto = (120 + 26 * len(lineas)) * e
        rect = pg.Rect(x, y, ancho, alto)
        pg.draw.rect(self.pt.sup, TARJETA_SEL if i == self.sel else TARJETA, rect, border_radius=int(16 * e))
        if i == self.sel:
            pg.draw.rect(self.pt.sup, ACENTO, rect, width=max(2, int(2 * e)), border_radius=int(16 * e))
        self.botones.append(Boton(rect, "", None, lambda i=i: setattr(self, "sel", i)))
        px, py = x + 24 * e, y + 18 * e
        w, _ = self._texto(p["nombre"], px, py, 32, TEXTO, True)
        if p.get("recomendado"):
            self._texto("RECOMENDADO", px + w + 16 * e, py + 8 * e, 18, ACENTO, True)
        py += 42 * e
        for ln in lineas:
            self._texto(ln, px, py, 22, TENUE)
            py += 26 * e
        py += 10 * e
        d = self.descargas.get(pid)
        bx = x + ancho - 24 * e
        if d is not None and d.is_alive():  # descargando
            barra = pg.Rect(px, py + 6 * e, ancho * 0.55, 18 * e)
            pg.draw.rect(self.pt.sup, (55, 62, 80), barra, border_radius=int(9 * e))
            lleno = barra.copy()
            lleno.width = max(1, int(barra.width * d.progreso))
            pg.draw.rect(self.pt.sup, AZUL, lleno, border_radius=int(9 * e))
            self._texto(f"{d.progreso * 100:.0f} %  {d.estado}", barra.right + 14 * e, py + 2 * e, 22, TEXTO)
            self._boton(bx, py - 6 * e, "Cancelar", ROJO, lambda: d.cancelar.set())
        elif paquetes.instalado(self.base, pid):
            self._texto("Instalado", px, py, 24, VERDE, True)
            borrar_txt = "¿Seguro? Borrar" if self.confirmar_borrar == pid else "Borrar"
            r1 = self._boton(bx, py - 6 * e, "Usar", VERDE, lambda: self._usar(pid))
            self._boton(r1.left - 12 * e, py - 6 * e, borrar_txt, ROJO, lambda: self._borrar(pid))
        else:
            estado = self.mensaje.get(pid) or f"No instalado · aprox. {p['tamano_mb']} MB"
            self._texto(estado, px, py, 24, ROJO if pid in self.mensaje else TENUE)
            self._boton(bx, py - 6 * e, "Descargar", AZUL, lambda: self._descargar(pid))
        return y + alto

    # ---------- acciones ----------
    def _descargar(self, pid):
        if pid in self.descargas and self.descargas[pid].is_alive():
            return
        self.mensaje.pop(pid, None)
        d = paquetes.Descarga(self.base, paquetes.por_id(pid))
        d.start()
        self.descargas[pid] = d

    def _borrar(self, pid):
        if self.confirmar_borrar == pid:
            paquetes.borrar(self.base, pid)
            self.confirmar_borrar = None
        else:
            self.confirmar_borrar = pid

    def _usar(self, pid):
        self.elegido = pid

    # ---------- bucle ----------
    def ejecutar(self):
        """Devuelve el id del paquete elegido, o None si se sale."""
        pg = self.pg
        pg.mouse.set_visible(True)
        self.pt._mouse_visible = True
        self.elegido = None
        while self.elegido is None:
            for pid, d in self.descargas.items():  # avisos de descargas terminadas
                if not d.is_alive() and d.error and pid not in self.mensaje:
                    self.mensaje[pid] = "No se pudo descargar: revisa la conexión a internet y reintenta."
            self.dibujar()
            self.pt.reloj.tick(30)
            for ev in pg.event.get():
                if ev.type == pg.QUIT:
                    return None
                if ev.type == pg.MOUSEBUTTONDOWN and ev.button == 1:
                    for b in reversed(self.botones):  # los botones van encima de su tarjeta
                        if b.rect.collidepoint(ev.pos):
                            if b.texto != "¿Seguro? Borrar" and b.texto != "Borrar":
                                self.confirmar_borrar = None
                            b.accion()
                            break
                if ev.type == pg.KEYDOWN:
                    pid = paquetes.CATALOGO[self.sel]["id"]
                    if ev.key == pg.K_ESCAPE:
                        return None
                    if ev.key == pg.K_UP:
                        self.sel = (self.sel - 1) % len(paquetes.CATALOGO)
                    elif ev.key == pg.K_DOWN:
                        self.sel = (self.sel + 1) % len(paquetes.CATALOGO)
                    elif ev.key in (pg.K_RETURN, pg.K_KP_ENTER):
                        if paquetes.instalado(self.base, pid):
                            self._usar(pid)
                        else:
                            self._descargar(pid)
                    elif ev.key == pg.K_DELETE and paquetes.instalado(self.base, pid):
                        self._borrar(pid)
        for d in self.descargas.values():  # no dejar descargas a medias al salir
            d.cancelar.set()
        self.config.write_text(json.dumps({"paquete": self.elegido}))
        return self.elegido


def pantalla_cargando(pantalla, texto):
    """Mensaje simple mientras se carga el cerebro en memoria."""
    if getattr(pantalla, "prueba", None):
        return
    pg = pantalla.pg
    pantalla.sup.fill(FONDO)
    f = pantalla._fuente(int(40 * min(pantalla.ancho / 1280, pantalla.alto / 800)))
    s = f.render(texto, True, TEXTO)
    pantalla.sup.blit(s, ((pantalla.ancho - s.get_width()) // 2, (pantalla.alto - s.get_height()) // 2))
    pg.display.flip()
    pg.event.pump()
