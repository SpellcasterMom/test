#!/usr/bin/env python3
"""
Project     : 6DOF arm (CLAW)
Author      : Vladimir V. Laykov aka Yago
Alma Mater  : Sevastopol State University
Date        : 2026-10-09
Last modify : 2026-10-09

Генератор презентации CLAW (12 слайдов) в фирменном шаблоне СевГУ.
Стратегия: клонируем слайды официального шаблона (мастер, лэйауты,
логотипы, шрифты сохраняются 1:1), подменяем только тексты,
добавляем чертёж robot1.jpg, таблицу экспериментов и notes.
Только stdlib (zipfile + xml.etree). Шаблон скачивается сам
(TEMPLATE_URL), в репо не коммитится.

Использование:
    python3 tools/make_deck.py            # -> docs/CLAW_presentation.pptx
Проверка: скрипт сам валидирует rels/контент-типы/разбор XML.
"""
import copy
import os
import sys
import urllib.request
import xml.etree.ElementTree as ET
import zipfile

TEMPLATE_URL = ("https://www.sevsu.ru/upload/iblock/68d/"
                "z7jmp0dq0k8xekmfxg298bmri3a4q44c/shablon_present_sevsu.pptx")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TMP = "/tmp/mywork/pptx"
TPL = os.path.join(TMP, "sevsu_template.pptx")
OUT = os.path.join(ROOT, "docs", "CLAW_presentation.pptx")
DRAWING = os.path.join(ROOT, "docs", "robot1.jpg")

P = "http://schemas.openxmlformats.org/presentationml/2006/main"
A = "http://schemas.openxmlformats.org/drawingml/2006/main"
R = "http://schemas.openxmlformats.org/officeDocument/2006/relationships"
RT = "http://schemas.openxmlformats.org/package/2006/relationships"
CT = "http://schemas.openxmlformats.org/package/2006/content-types"
ET.register_namespace("", P)
ET.register_namespace("a", A)
ET.register_namespace("r", R)


def qn(ns, tag):
    return "{%s}%s" % ({"p": P, "a": A, "r": R}[ns], tag)


def emu(px, dpi=96):
    return int(px * 914400 / dpi)


# ---------------------------------------------------------------- контент
# (заголовок, [буллиты], заметки докладчику)
SLIDES = [
    ("Шестиосевой манипулятор CLAW",
     ["Дипломный проект",
      "Лайков В.В., группа УТС/б-23-1-о",
      "Руководитель: Липко И.Ю.",
      "Севастопольский государственный университет"],
     "Титул. Представление: кто, что, где. ~30 секунд."),
    ("Цель работы",
     ["Рабочий настольный манипулятор, а не макет",
      "Честная математика вместо магии: модель, ворота, отказ с причиной",
      "Двойник на Python: проверка без железа"],
     "Ставка проекта: воспроизводимость и честные цифры. ~40 секунд."),
    ("Железо",
     ["ESP32, 2 ядра: движение + дисплей",
      "7 серв через PCA9685: P1–P3 270° ±135°, P4–P6 180° ±90°, P7 0–100%",
      "IMU BMI160 (гироскоп + акселерометр), дисплей ST7789, пульт-джоггер"],
     "Перечислить узлы, показать фото руки (вставить вручную). ~40 секунд."),
    ("Прямая кинематика: цепочка DH",
     ["Звенья 134 / 165 / 161 / 0 / 53+54 / 27 мм, палец 105 мм",
      "Запястье НЕ сферическое — это усложняет IK",
      "Нули: TCP (132, 53, 514), инструмент +X (проверено рулеткой)"],
     "Чертёж robot1.jpg уже встроен справа. Указать на несоосность. ~40 секунд."),
    ("Обратная кинематика",
     ["Геометрические сиды + доводка Левенберга–Марквардта",
      "Выбор min-residual со штрафами (непрерывность, высота)",
      "Ворота годности: 10 мм / 15°"],
     "Ключевой слайд математики, без формул — только схема. ~45 секунд."),
    ("Ворота: хуже — отказ, а не движение",
     ["Хуже 10 мм / 15° — честный ERROR с причиной в M115",
      "Пример: точка (100,100,100) — 2.2 мм / 16.5°, отказ",
      "Нет неточного движения как класса"],
     "Философия проекта. Пример из живых логов. ~40 секунд."),
    ("Планировщик объезда",
     ["Via-точки у граней запретных боксов + RRT (120 узлов)",
      "Детерминирован: повтор даёт то же дерево",
      "Живой пример: перелёт через ящик WPS: 3"],
     "Схема объезда. ~40 секунд."),
    ("Дисплей, команды, IMU",
     ["TCP + Roll/Pitch/Yaw, кружки PCA/GYR, версия в шапке",
      "G0/G1, M104/M108, M115-разбор, M119-дамп IMU",
      "Логгер 200 Гц: кольцо последнего движения"],
     "Показать фото экрана. ~40 секунд."),
    ("Двойник на Python",
     ["Зеркало прошивки 1:1: тот же DLS, те же ворота",
      "Трекинг-модель совпала с железом до 0.1° (24/24)",
      "Вся кинематика сначала доказывается здесь"],
     "Про двойника коротко — это инструмент, не результат. ~35 секунд."),
    ("Эксперименты", None,  # таблица ниже
     "Главный слайд с цифрами. Читать построчно. ~60 секунд."),
    ("Выводы и дальше",
     ["Работает: T1 0.5/12.2, круг 24/24 пик 0.3°, свип без решений под столом",
      "Измерено: стена 120–150, tilt-профили, bias гироскопа",
      "Дальше: самоколлизии-капсулы, BMI160"],
     "Честно: что сделано и что открыто. ~40 секунд."),
    ("Спасибо за внимание!",
     ["Вопросы",
      "Лайков В.В., УТС/б-23-1-о",
      "Репозиторий, MATH.docx, двойник — в приложении"],
     "Финал. ~20 секунд."),
]

TABLE = {
    "headers": ["Этап", "dPos", "dOri", "Итог"],
    "rows": [
        ["T1 (200,200,200)", "0.5 мм", "12.2°", "OK, ветвь [0]"],
        ["Круг tilt, 24 точки", "≤0.9 мм", "пик 0.3°", "24/24 OK"],
        ["Свип 168 точек", "—", "—", "67 принято, 0 под столом"],
        ["Отказы", "2.2 мм", "16.5–19°", "Честный ERROR + причина"],
    ],
}


def ensure_template():
    if os.path.exists(TPL):
        return TPL
    os.makedirs(TMP, exist_ok=True)
    print("скачиваю шаблон СевГУ...")
    urllib.request.urlretrieve(TEMPLATE_URL, TPL)
    return TPL


def tx_rpr_copy(src_rpr):
    return copy.deepcopy(src_rpr)


def set_box_text(sp, paras, base_sz=None):
    """paras: [(text, size|None)]. Пересобирает txBody, стиль из первого rPr."""
    txb = sp.find(qn("p", "txBody"))
    first_rpr = None
    for r in txb.iter(qn("a", "r")):
        first_rpr = r.find(qn("a", "rPr"))
        if first_rpr is not None:
            break
    first_ppr = txb.find(qn("a", "p"))
    first_ppr = first_ppr.find(qn("a", "pPr")) if first_ppr is not None else None
    for p in list(txb.findall(qn("a", "p"))):
        txb.remove(p)
    for text, sz in paras:
        p = ET.SubElement(txb, qn("a", "p"))
        if first_ppr is not None:
            p.append(copy.deepcopy(first_ppr))
            # добавляем маркер-буллит, если его не было
            ppr = p.find(qn("a", "pPr"))
            if ppr.find(qn("a", "buChar")) is None and p.find(qn("a", "buNone")) is None:
                bu = ET.SubElement(ppr, qn("a", "buChar"))
                bu.set("char", "•")
        r = ET.SubElement(p, qn("a", "r"))
        rpr = tx_rpr_copy(first_rpr) if first_rpr is not None else ET.Element(qn("a", "rPr"))
        if sz:
            rpr.set("sz", str(sz))
        r.append(rpr)
        t = ET.SubElement(r, qn("a", "t"))
        t.text = text
    return txb


def text_shapes(root):
    out = []
    for sp in root.iter(qn("p", "sp")):
        if sp.find(qn("p", "txBody")) is not None:
            off = sp.find(qn("p", "spPr") + "/" + qn("a", "xfrm") + "/" + qn("a", "off"))
            ext = sp.find(qn("p", "spPr") + "/" + qn("a", "xfrm") + "/" + qn("a", "ext"))
            y = int(off.get("y")) if off is not None else 0
            area = (int(ext.get("cx")) * int(ext.get("cy"))) if ext is not None else 0
            out.append((sp, y, area))
    return out


def build():
    ensure_template()
    zin = zipfile.ZipFile(TPL)
    parts = {n: zin.read(n) for n in zin.namelist()}
    slide1 = ET.fromstring(parts["ppt/slides/slide1.xml"])
    slide2 = ET.fromstring(parts["ppt/slides/slide2.xml"])
    slide3 = ET.fromstring(parts["ppt/slides/slide3.xml"])

    def boxes(root):
        ts = text_shapes(root)
        ts.sort(key=lambda t: t[1])
        title = ts[0][0]
        body = max(ts[1:], key=lambda t: t[2])[0] if len(ts) > 1 else ts[0][0]
        return title, body

    t1_title, _ = boxes(slide1)
    t2_title, t2_body = boxes(slide2)

    new_slides = []  # (filename, root, base_rels_name)
    # --- S1: титул ---
    r = copy.deepcopy(slide1)
    tt, _ = boxes(r)
    set_box_text(tt, [(SLIDES[0][0], 3200)] + [(t, 2000) for t in SLIDES[0][1]])
    new_slides.append(("ppt/slides/slide1.xml", r, "slide1"))
    # --- S2..S11: контент ---
    for idx in range(1, 11):
        r = copy.deepcopy(slide2)
        tt, bb = boxes(r)
        set_box_text(tt, [(SLIDES[idx][0], 2800)])
        if SLIDES[idx][1] is None:
            set_box_text(bb, [("(таблица экспериментов — см. ниже)", 1800)])
        else:
            set_box_text(bb, [(t, 2000) for t in SLIDES[idx][1]])
        new_slides.append(("ppt/slides/slide%d.xml" % (idx + 1), r, "slide2"))
    # --- S12: финал ---
    r = copy.deepcopy(slide3)
    tt, _ = boxes(r)
    set_box_text(tt, [(SLIDES[11][0], 3200)] + [(t, 2000) for t in SLIDES[11][1]])
    new_slides.append(("ppt/slides/slide12.xml", r, "slide3"))

    for name, root, _ in new_slides:
        parts[name] = ET.tostring(root, encoding="utf-8", xml_declaration=True)

    # --- чертёж robot1.jpg на слайд 4 (FK) ---
    try:
        from PIL import Image
        im = Image.open(DRAWING).convert("RGB")
        im.thumbnail((1400, 1400))
        import io as _io
        buf = _io.BytesIO()
        im.save(buf, "JPEG", quality=82)
        jpg = buf.getvalue()
    except Exception:
        with open(DRAWING, "rb") as f:
            jpg = f.read()
    parts["ppt/media/image3.jpg"] = jpg
    # уменьшаем текстовый бокс слайда 4 и кладём картинку справа
    r4 = ET.fromstring(parts["ppt/slides/slide4.xml"])
    _, bb = boxes(r4)
    xfrm = bb.find(qn("p", "spPr") + "/" + qn("a", "xfrm"))
    ext = xfrm.find(qn("a", "ext"))
    ext.set("cx", str(int(int(ext.get("cx")) * 0.55)))
    # пропорции картинки
    try:
        from PIL import Image as _I
        import io as _io2
        _im = _I.open(_io2.BytesIO(jpg))
        iw, ih = _im.size
    except Exception:
        iw, ih = (1400, 1000)
    pw, ph = 3000000, int(3000000 * ih / iw)
    px, py = 5600000, 1100000
    pic = ET.fromstring(
        '<p:pic xmlns:p="%s" xmlns:a="%s" xmlns:r="%s">'
        '<p:nvPicPr><p:cNvPr id="200" name="robot1"/>'
        '<p:cNvPicPr/><p:nvPr/></p:nvPicPr><p:blipFill>'
        '<a:blip r:embed="rId9"/><a:stretch><a:fillRect/></a:stretch>'
        '</p:blipFill><p:spPr><a:xfrm><a:off x="%d" y="%d"/>'
        '<a:ext cx="%d" cy="%d"/></a:xfrm><a:prstGeom prst="rect">'
        '<a:avLst/></a:prstGeom></p:spPr></p:pic>' % (P, A, R, px, py, pw, ph))
    # вставляем картинку в spTree после текстовых боксов
    sptree = r4.find(qn("p", "cSld") + "/" + qn("p", "spTree"))
    sptree.append(pic)
    parts["ppt/slides/slide4.xml"] = ET.tostring(r4, encoding="utf-8", xml_declaration=True)

    # --- таблица экспериментов на слайд 10 ---
    r10 = ET.fromstring(parts["ppt/slides/slide10.xml"])
    _, bb = boxes(r10)
    xfrm = bb.find(qn("p", "spPr") + "/" + qn("a", "xfrm"))
    off = xfrm.find(qn("a", "off"))
    tx, ty = int(off.get("x")), int(off.get("y"))
    # таблицу ставим под текстовый бокс: сдвигаем бокс-заглушку в маленький заголовок
    set_box_text(bb, [("Итоги замеров (M115-логи):", 1800)])
    tbl = build_table(tx, ty + 700000, 7600000)
    sptree = r10.find(qn("p", "cSld") + "/" + qn("p", "spTree"))
    sptree.append(tbl)
    parts["ppt/slides/slide10.xml"] = ET.tostring(r10, encoding="utf-8", xml_declaration=True)

    # --- rels слайдов: копируем базовые + notes + image3 ---
    base_rels = {}
    for b in ("slide1", "slide2", "slide3"):
        base_rels[b] = ET.fromstring(parts["ppt/slides/_rels/%s.xml.rels" % b])
    # существующие rId в базовых rels
    import re as _re
    def max_rid(root):
        m = 0
        for el in root.iter():
            v = el.get("{%s}id" % R)
            if v and v.startswith("rId"):
                try:
                    m = max(m, int(v[3:]))
                except ValueError:
                    pass
        return m
    notes_ids = {}  # slide idx -> rId ноты
    for i in range(12):
        base = "slide1" if i == 0 else ("slide3" if i == 11 else "slide2")
        rr = copy.deepcopy(base_rels[base])
        used = max_rid(rr)
        # notes
        nr = "rId%d" % (used + 1)
        used += 1
        e = ET.SubElement(rr, "{%s}Relationship" % RT)
        e.set("Id", nr)
        e.set("Type", "http://schemas.openxmlformats.org/officeDocument/2006/relationships/notesSlide")
        e.set("Target", "notesSlide%d.xml" % (i + 1))
        notes_ids[i] = nr
        if i == 3:  # чертёж на слайде 4
            ir = "rId%d" % (used + 1)
            e2 = ET.SubElement(rr, "{%s}Relationship" % RT)
            e2.set("Id", ir)
            e2.set("Type", "http://schemas.openxmlformats.org/officeDocument/2006/relationships/image")
            e2.set("Target", "../media/image3.jpg")
            # pic в слайде ссылается на rId9 — правим под выданный
            sld = ET.fromstring(parts["ppt/slides/slide4.xml"])
            for blip in sld.iter(qn("a", "blip")):
                blip.set("{%s}embed" % R, ir)
            parts["ppt/slides/slide4.xml"] = ET.tostring(sld, encoding="utf-8", xml_declaration=True)
        parts["ppt/slides/_rels/slide%d.xml.rels" % (i + 1)] = ET.tostring(
            rr, encoding="utf-8", xml_declaration=True)

    # --- notesSlides ---
    for i in range(12):
        parts["ppt/notesSlides/notesSlide%d.xml" % (i + 1)] = notes_xml(SLIDES[i][2], i + 1)
        nr = ET.Element("{%s}Relationships" % RT)
        e1 = ET.SubElement(nr, "{%s}Relationship" % RT)
        e1.set("Id", "rId1")
        e1.set("Type", "http://schemas.openxmlformats.org/officeDocument/2006/relationships/notesMaster")
        e1.set("Target", "../notesMasters/notesMaster1.xml")
        e2 = ET.SubElement(nr, "{%s}Relationship" % RT)
        e2.set("Id", "rId2")
        e2.set("Type", "http://schemas.openxmlformats.org/officeDocument/2006/relationships/slide")
        e2.set("Target", "../slides/slide%d.xml" % (i + 1))
        parts["ppt/notesSlides/_rels/notesSlide%d.xml.rels" % (i + 1)] = ET.tostring(
            nr, encoding="utf-8", xml_declaration=True)

    # --- presentation.xml: 12 слайдов ---
    pres = ET.fromstring(parts["ppt/presentation.xml"])
    lst = pres.find(qn("p", "sldIdLst"))
    for s in list(lst.findall(qn("p", "sldId"))):
        lst.remove(s)
    pres_rels = ET.fromstring(parts["ppt/_rels/presentation.xml.rels"])
    # чистим старые slide-rel (rId2..rId4), шрифты/мастер не трогаем
    for rel in list(pres_rels):
        t = rel.get("Target") or ""
        if "/slides/slide" in t:
            pres_rels.remove(rel)
    rid = 19
    for i in range(12):
        e = ET.SubElement(pres_rels, "{%s}Relationship" % RT)
        e.set("Id", "rId%d" % rid)
        e.set("Type", "http://schemas.openxmlformats.org/officeDocument/2006/relationships/slide")
        e.set("Target", "slides/slide%d.xml" % (i + 1))
        s = ET.SubElement(lst, qn("p", "sldId"))
        s.set("id", str(256 + i))
        s.set("{%s}id" % R, "rId%d" % rid)
        rid += 1
        # notes rel
        e2 = ET.SubElement(pres_rels, "{%s}Relationship" % RT)
        e2.set("Id", "rId%d" % (rid + 11))
        e2.set("Type", "http://schemas.openxmlformats.org/officeDocument/2006/relationships/notesSlide")
        e2.set("Target", "notesSlides/notesSlide%d.xml" % (i + 1))
    rid += 12
    parts["ppt/_rels/presentation.xml.rels"] = ET.tostring(pres_rels, encoding="utf-8", xml_declaration=True)
    parts["ppt/presentation.xml"] = ET.tostring(pres, encoding="utf-8", xml_declaration=True)

    # --- [Content_Types]: слайды 4..12 + notes ---
    cts = ET.fromstring(parts["[Content_Types].xml"])
    have = set()
    for el in cts.iter():
        if el.tag.endswith("}Override"):
            have.add(el.get("PartName"))
    for i in range(4, 13):
        p = "/ppt/slides/slide%d.xml" % i
        if p not in have:
            e = ET.SubElement(cts, "{%s}Override" % CT)
            e.set("PartName", p)
            e.set("ContentType", "application/vnd.openxmlformats-officedocument.presentationml.slide+xml")
    for i in range(1, 13):
        p = "/ppt/notesSlides/notesSlide%d.xml" % i
        e = ET.SubElement(cts, "{%s}Override" % CT)
        e.set("PartName", p)
        e.set("ContentType", "application/vnd.openxmlformats-officedocument.presentationml.notesSlide+xml")
    parts["[Content_Types].xml"] = ET.tostring(cts, encoding="utf-8", xml_declaration=True)

    # --- docProps/app: счётчик слайдов, TitlesOfParts долой ---
    try:
        app = ET.fromstring(parts["docProps/app.xml"])
        for el in app.iter():
            if el.tag.endswith("}Slides"):
                el.text = "12"
            if el.tag.endswith("}TitlesOfParts"):
                # удаляем целиком (PowerPoint пересоздаст)
                pass
        # убираем TitlesOfParts + HeadingPairs аккуратно
        for parent in list(app.iter()):
            for ch in list(parent):
                if ch.tag.endswith("}TitlesOfParts") or ch.tag.endswith("}HeadingPairs"):
                    parent.remove(ch)
        parts["docProps/app.xml"] = ET.tostring(app, encoding="utf-8", xml_declaration=True)
    except KeyError:
        pass

    # --- запись ---
    with zipfile.ZipFile(OUT, "w", zipfile.ZIP_DEFLATED) as z:
        for n, d in parts.items():
            z.writestr(n, d)
    print("записано:", OUT, os.path.getsize(OUT), "байт")


def build_table(x, y, w):
    """graphicFrame с таблицей экспериментов 4x5."""
    cols = [3000000, 1500000, 1500000, 1600000]
    rows = [["Этап", "dPos", "dOri", "Итог"]] + TABLE["rows"]
    rh = 420000
    h = rh * len(rows)
    g = ET.fromstring(
        '<p:graphicFrame xmlns:p="%s" xmlns:a="%s" xmlns:r="%s">'
        '<p:nvGraphicFramePr><p:cNvPr id="300" name="Table"/>'
        '<p:cNvGraphicFramePr><a:graphicFrameLocks noChangeAspect="0"/></p:cNvGraphicFramePr>'
        '<p:nvPr/></p:nvGraphicFramePr>'
        '<p:xfrm><a:off x="%d" y="%d"/><a:ext cx="%d" cy="%d"/></p:xfrm>'
        '<a:graphic><a:graphicData uri="http://schemas.openxmlformats.org/drawingml/2006/table">'
        '<a:tbl><a:tblPr firstRow="1" bandRow="0"><a:tableStyleId>'
        '{5940675A-B579-460E-94D1-542E71E4F335}</a:tableStyleId></a:tblPr>'
        '<a:tblGrid>%s</a:tblGrid>%s</a:tbl>'
        '</a:graphicData></a:graphic></p:graphicFrame>'
        % (P, A, R, x, y, w, h,
           "".join('<a:gridCol w="%d"/>' % c for c in cols),
           "".join(table_row(r, rh, i == 0) for i, r in enumerate(rows))))
    return g


def table_row(cells, rh, header):
    tds = []
    for c in cells:
        tds.append(
            '<a:tc><a:txBody><a:bodyPr wrap="square"><a:spAutoFit/></a:bodyPr>'
            '<a:lstStyle/><a:p><a:r><a:rPr lang="ru-RU" sz="1600"%s>'
            '<a:solidFill><a:srgbClr val="%s"/></a:solidFill>'
            '<a:latin typeface="Myriad Pro" pitchFamily="34" charset="0"/>'
            '</a:rPr><a:t>%s</a:t></a:r></a:p></a:txBody>'
            '<a:tcPr><a:lnL w="12700"><a:solidFill><a:srgbClr val="44546A"/>'
            '</a:solidFill></a:lnL><a:lnR w="12700"><a:solidFill>'
            '<a:srgbClr val="44546A"/></a:solidFill></a:lnR>'
            '<a:lnT w="12700"><a:solidFill><a:srgbClr val="44546A"/>'
            '</a:solidFill></a:lnT><a:lnB w="12700"><a:solidFill>'
            '<a:srgbClr val="44546A"/></a:solidFill></a:lnB></a:tcPr></a:tc>'
            % (" b=\"1\"" if header else "", "FFFFFF" if header else "1F497D", c))
    return "<a:tr h=\"%d\">%s</a:tr>" % (rh, "".join(tds))


def notes_xml(text, idx):
    return ET.tostring(ET.fromstring(
        '<p:notesSlide xmlns:p="%s" xmlns:a="%s" xmlns:r="%s">'
        '<p:cSld name="notesSlide%d"><p:spTree>'
        '<p:nvGrpSpPr><p:cNvPr id="1" name=""/>'
        '<p:cNvGrpSpPr/><p:nvPr/></p:nvGrpSpPr>'
        '<p:grpSpPr><a:xfrm><a:off x="0" y="0"/><a:ext cx="0" cy="0"/>'
        '<a:chOff x="0" y="0"/><a:chExt cx="0" cy="0"/></a:xfrm></p:grpSpPr>'
        '<p:sp><p:nvSpPr><p:cNvPr id="2" name="Slide Image"/>'
        '<p:cNvSpPr><a:spLocks noChangeAspect="1"/></p:cNvSpPr>'
        '<p:nvPr><p:ph type="sldImg"/></p:nvPr></p:nvSpPr>'
        '<p:spPr/><p:txBody><a:bodyPr/><a:lstStyle/><a:p><a:endParaRPr/>'
        '</a:p></p:txBody></p:sp>'
        '<p:sp><p:nvSpPr><p:cNvPr id="3" name="Notes"/>'
        '<p:cNvSpPr><a:spLocks noChangeAspect="1"/></p:cNvSpPr>'
        '<p:nvPr><p:ph type="body" idx="1"/></p:nvPr></p:nvSpPr>'
        '<p:spPr/><p:txBody><a:bodyPr/><a:lstStyle/>'
        '<a:p><a:r><a:rPr lang="ru-RU" sz="1200"/>'
        '<a:t>NOTES</a:t></a:r></a:p></a:txBody></p:sp>'
        '</p:spTree></p:cSld>'
        '<p:clrMapOvr><a:overrideClrMapping bg1="lt1" tx1="dk1" '
        'bg2="lt2" tx2="dk2" accent1="accent1" accent2="accent2" '
        'accent3="accent3" accent4="accent4" accent5="accent5" '
        'accent6="accent6" hlink="hlink" folHlink="folHlink"/>'
        '</p:clrMapOvr></p:notesSlide>' % (P, A, R, idx)
        ).replace(b"NOTES", text.encode("utf-8")),
        encoding="utf-8", xml_declaration=True)


def main():
    build()


if __name__ == "__main__":
    main()
