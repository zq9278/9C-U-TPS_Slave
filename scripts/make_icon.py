"""Build the product-specific ICO/PNG from code-native vector geometry."""
from pathlib import Path
from PIL import Image, ImageDraw
ROOT=Path(__file__).resolve().parents[1]
def main():
    size=1024;im=Image.new('RGBA',(size,size));d=ImageDraw.Draw(im)
    d.rounded_rectangle((32,32,992,992),radius=216,fill='#15566b')
    # Same eye + pulse geometry as assets/tps130.svg, drawn at 4x resolution.
    def bez(p0,p1,p2,p3):
        return [tuple((1-t)**3*p0[i]+3*(1-t)**2*t*p1[i]+3*(1-t)*t*t*p2[i]+t**3*p3[i] for i in (0,1)) for t in [j/80 for j in range(81)]]
    eye=bez((172,512),(280,344),(404,288),(512,288))+bez((512,288),(620,288),(744,344),(852,512))+bez((852,512),(744,680),(620,736),(512,736))+bez((512,736),(404,736),(280,680),(172,512))
    d.line(eye,fill='#e7faf8',width=48,joint='curve')
    pulse=[(232,528),(364,528),(424,392),(512,664),(600,404),(656,528),(792,528)]
    d.line(pulse,fill='#f3b56d',width=44,joint='curve')
    for x,y in pulse:d.ellipse((x-22,y-22,x+22,y+22),fill='#f3b56d')
    d.ellipse((736,180,816,260),fill='#f3b56d')
    assets=ROOT/'Tools/Flasher/assets';assets.mkdir(parents=True,exist_ok=True)
    im.resize((256,256),Image.Resampling.LANCZOS).save(assets/'tps130.png')
    im.save(assets/'tps130.ico',sizes=[(16,16),(24,24),(32,32),(48,48),(64,64),(128,128),(256,256)])
    print(assets/'tps130.ico')
if __name__=='__main__':main()
