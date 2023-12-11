# ros2_course

## Bevezető
Tartalmaz egy-két órai gyakorlat anyagát, illetve a kötlezeő programot. A kötelező program témájának a TurtleSim-et választottam.

### Fraktált
---
A fraktálok végtelenül komplex geometriai alakzatok, amelyek két gyakori, jellemző tulajdonsággal rendelkeznek. Az első, hogy a tradicionálisan a geometria által vizsgált, bizonyos értelemben véve „egyszerűbb” [...] alakzatokkal ellentétben [...] a fraktálok határoló vonalai vagy -felületei végtelenül „gyűröttek”, illetve „szakadásosak” (szakkifejezéssel, nem-differenciálhatóak). A második gyakori jellemzőjük az [...] önhasonlóság.[1]

![Fraktált gif](https://media1.giphy.com/media/M4ofIAGWCSlIk/giphy.gif)

#### Sierpinski-háromszög
A Sierpiński-háromszög Wacław Sierpiński lengyel matematikus által megtalált fraktál, amely úgy áll elő, hogy egy szabályos háromszögből elhagyjuk az oldalfelező pontok összekötésével nyert belső háromszöget, majd az így maradt három háromszögre rekurzívan alkalmazzuk ugyanezt az eljárást.[2]

![Sierpinski-háromszög](https://upload.wikimedia.org/wikipedia/commons/2/27/SierpinskiTriangle-ani-0-7.gif)

#### Püthagorasz-fa
A Pitagorasz-fa négyzetekből épül fel, amik úgy helyezkednek el, ahogy azt a Pitagorasz-tétel ábrázolásai mutatják.

![Püthagorasz-fa](https://upload.wikimedia.org/wikipedia/commons/a/a2/PythagorasTree.png)

#### Newton-fraktál
Egy másik fraktál a Newton-fraktál, ami Newton-módszerrel számítható.

![Newton-fraktál](https://upload.wikimedia.org/wikipedia/commons/9/9a/Newtroot_1_0_0_m1.png)

## ToDo list
- [ ] Basic: Arányos szabályozó implementálása
- [ ] Advanced: Szöveg vagy fraktált kirajzolása
- [ ] Epic: Nyűgözz le



## USAGE

How to *build* and use the package

	cd ~/ros2_ws
	colcon build --symlink-install


---
## Források
[1] https://hu.wikipedia.org/wiki/Fraktál
[2] https://hu.wikipedia.org/wiki/Sierpiński-háromszög
