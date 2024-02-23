import sys
from PIL import Image

assets = [f for f in sys.argv[2:]]

f = open(sys.argv[1], "w+")

f.write("#include \"../src/assets.h\"\n")

for asset in assets:
    if asset.endswith(".png"):
        name = asset.replace("assets", "asset_texture").replace("/", "_").replace(".png", "")
        img = Image.open(asset)

        # write image data
        f.write(f"ubyte {name}_data[] = " + "{")
        for y in range(img.height):
            for x in range(img.width):
                r, g, b, a = img.getpixel((x, y))
                f.write(f"{hex(r)}, {hex(g)}, {hex(b)}, {hex(a)}, ")
        f.write("};\n")

        img.close()

        # write asset struct
        f.write(f"asset_image {name} = " + "{" + f"{img.width}, {img.height}, 4, {name}_data" + "};\n")


f.write("extern struct asset_storage builtin_assets[ASSET_COUNT];\n")
f.write("errcode assets_init(void)\n{\n")

for asset in assets:
    if asset.endswith(".png"):
        name = asset.replace("assets", "asset_texture").replace("/", "_").replace(".png", "")
        f.write(f"builtin_assets[{name.upper()}] = *(struct asset_storage *)&{name};\n")

        
f.write("return ERR_OK;\n}\n")

f.close()
