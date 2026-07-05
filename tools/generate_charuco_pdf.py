import base64
import re
import sys
from io import BytesIO
from pathlib import Path

from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.lib.utils import ImageReader
from reportlab.pdfgen import canvas


def main() -> int:
    if len(sys.argv) != 3:
        print("Usage: generate_charuco_pdf.py <input.svg> <output.pdf>")
        return 1

    input_path = Path(sys.argv[1])
    output_path = Path(sys.argv[2])
    svg = input_path.read_text(encoding="utf-8")
    match = re.search(r"data:image/png;base64,([^\"]+)", svg)
    if not match:
        raise RuntimeError("The SVG does not contain an embedded PNG board image")

    board_png = base64.b64decode(match.group(1))
    output_path.parent.mkdir(parents=True, exist_ok=True)

    page_width, page_height = A4
    board_width = 160 * mm
    board_height = 220 * mm
    board_x = (page_width - board_width) / 2
    board_y = (page_height - board_height) / 2

    document = canvas.Canvas(str(output_path), pagesize=A4, pageCompression=1)
    document.setTitle("CubeVision ChArUco Calibration Board")
    document.setAuthor("CubeVision")
    document.drawImage(
        ImageReader(BytesIO(board_png)),
        board_x,
        board_y,
        width=board_width,
        height=board_height,
        preserveAspectRatio=True,
        anchor="c",
    )
    document.showPage()
    document.save()
    print(f"Generated {output_path} on A4 at 100% physical scale")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
