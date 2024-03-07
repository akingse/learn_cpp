

## 自制目录 [name](#name)

- [希腊字母](#希腊字母)



### 欧拉公式 Euler Formula

$$
e^{ix}=cosx+isinx\\
e^{iπ}+1=0
$$

$e^{i\pi}+1=0$

$e^{i\pi}+1=\!0$

$e^{i\pi}+1=\:0$

$e^{i\pi}+1=\;0$

$e^{i\pi}+1=\,0$

$e^{i\pi}+1=\>0$

### 数学语法

```
$\,$ 3/18em   
$\:$  4/18em  
$\;$ 5/18em 
$\quad$ 1em 
$\qquad$ 2m 
$\!$ -3/18em
```

### 空格

| 格式         | 代码           | 样例                                                         | 说明          |
| ------------ | -------------- | ------------------------------------------------------------ | ------------- |
| 两个quad空格 | `$a \qquad b$` | ![a \qquad b](https://math.jianshu.com/math?formula=a%20%5Cqquad%20b) | 两个*m*的宽度 |
| 一个quad空格 | `$a \quad b$`  | ![a \quad b](https://math.jianshu.com/math?formula=a%20%5Cquad%20b) | 一个*m*的宽度 |
| 大空格       | `$ a\ b$`      | ![a\ b](https://math.jianshu.com/math?formula=a%5C%20b)      | 1/3m宽度      |
| 中等空格     | `$ a\;b$`      | ![a\;b](https://math.jianshu.com/math?formula=a%5C%3Bb)      | 2/7m宽度      |
| 小空格       | `$a\,b$`       | ![a\,b](https://math.jianshu.com/math?formula=a%5C%2Cb)      | 1/6m宽度      |
| 没有空格     | `$ab$`         | ![ab](https://math.jianshu.com/math?formula=ab)              | 1/3m宽度      |
| 紧帖         | `$a\!b$`       | ![a\!b](https://math.jianshu.com/math?formula=a%5C!b)        | 缩进1/6m宽度  |



```
欧拉公式
“上帝公式”、“最伟大的数学公式”、“数学家的宝藏”

0到1是从无到有，时空维度角度考虑也可以理解为从无到一维;
1到pi是线到圆，可以理解为一维到二维;
pi到e是圆从圆心提溜起来的那个螺旋式上升曲线，可以理解为二维到三维;
欧拉公式把无，一维，二维，三维联系了起来

美学的极致，强迫症一脸满足；$e^{i\pi}+1=0$

哈密尔顿四元数
```



## 公式语法

发现不同的编辑器，公式语法显示略有不同，跨平台就很麻烦；注意，以下公式使用typora书写，为了更好的体验，请使用typora打开。

[quote](https://blog.csdn.net/u014630987/article/details/70156489)

行内和行外，显示略有不同，typora不支持；比如

```latex
$ \sum_{i=0}^{n}i^2 $
$$ \sum_{i=0}^{n}i^2 $$
```



```
字符: # $ % & ~ _ ^ \ { }有特殊意义，需要表示这些字符时，需要转义 \
```

\boxed 加框
$$
\boxed{E=mc^2}
$$
开方
$$
\sqrt[开方次数，不加中括号默认为2]{开方公式}
$$
\quad表示添加空格
$$
y=x\quad\quad表示添加空格
$$
\frac 分数

分数用\frac表示，字号工具环境设置，\dfrac命令设置为独立公式中的大小，\tfrac则把字号设置为行间公式中的大小。
$$
\frac{1}{2} \quad \dfrac{1}{2} \quad \tfrac{1}{2}
$$
**运算符**

\+ - * / = 直接输入，特殊运算则用以下特殊命令
`$$ \pm\; \times\; \div\; \cdot\; \cap\; \cup\; \geq\; \leq\; \neq\; \approx\; \equiv $$`
$$

$$

$$
\pm \times \div \cdot \cap \cup \geq \leq \neq \approx \equiv
$$

$$
\pm\; \times\; \div\; \cdot\; \cap\; \cup\; \geq\; \leq\; \neq\; \approx\; \equiv\;
$$

$$
\pm\ \times\ \div\ \cdot\ \cap\ \cup\ \geq\ \leq\ \neq\ \approx\ \equiv\
$$

和、积、极限、积分等运算符用\sum, \prod, \lim, \int,这些公式在行内公式被压缩，以适应行高，可以通过\limits和\nolimits命令显示制动是否压缩。
`$ \sum\; \prod\; \lim\; \int\; $` 
$$
\sum\; \prod\; \lim\; \int\; x\to0
$$
多重积分使用如下形式\int、\iint、\iiint、\iiiint、\idotsint，例如
$$
\int\int\quad \int\int\int\quad
   \int\int\int\int\quad \int\dots\int

	\iint\quad \iiint\quad \iiiint\quad \idotsint
$$

箭头
$$
$ \leftarrow $ => ←　　　　 　 $ \rightarrow $ 表示→\\
$ \leftrightarrow $表示　↔　　　 $\Leftarrow$表示⇐\\
$\Rightarrow$ 表示⇒ $ \Leftrightarrow$表示⇔\\
$ \longleftarrow $表示 ⟵ $ \longleftarrow $表示⟵\\
$longleftrightarrow$表示⟷ $ \Longleftarrow $表示⟸\\
$\Longrightarrow$表示⟹ $\Longleftrightarrow$表示⟺\\
\xleftarrow和\xrightarrow可根据内容自动调整\\
$$
**分隔符**

括号用`() [] \{\} \langle \rangle`
$$
() [] \{\} \langle \rangle || \|\|
$$

$$
\Bigg(\bigg(\Big(\big((x)\big)\Big)\bigg)\Bigg)\quad
\Bigg[\bigg[\Big[\big[[x]\big]\Big]\bigg]\Bigg]\quad
\Bigg\{\bigg\{\Big\{\big\{\{x\}\big\}\Big\}\bigg\}\Bigg\}

\Bigg \langle \bigg \langle \Big \langle\big\langle\langle x 
\rangle \big \rangle\Big\rangle\bigg\rangle\Bigg\rangle \quad
\Bigg\lvert\bigg\lvert\Big\lvert\big\lvert\lvert x \rvert\big\rvert\Big\rvert\bigg\rvert\Bigg\rvert\quad
\Bigg\lVert\bigg\lVert\Big\lVert\big\lVert\lVert x \rVert\big\rVert\Big\rVert\bigg\rVert\Bigg\rVert
$$



**省略号**

省略号用 `\dots \cdots \vdots \ddots`表示 ，`\dots和\cdots`的纵向位置不同，前者一般用于有下标的序列
$$
0\dots \\
0\cdots \\
0\vdots \\
0\ddots \\
0\ldots \\
$$

$$
x_1, x_2, \dots, x_n\quad 1,2,\cdots,n\quad \vdots\quad \ddots
$$

间距 \转义
$$
间距控制，略有不同\\
1\!2  \\
1\,2\\
1\:2 \\
1\;2 \\
1\quad2 \\
1\qquad2 \\

|\|  \\
\#  \\
$\$  \\
注释\%  \\
对齐符号\&  \\
下标\_  \\
*\ast  \\
$$
矩阵
$$
\begin{array}{ccc}
x_1 & x_2 & \dots \\
x_3 & x_ 4& \dots \\
\vdots & \vdots & \ddots
\end{array}
$$

$$
\begin{pmatrix} a & b\\ c & d \\ \end{pmatrix} \quad
\begin{bmatrix} a & b \\ c & d \\ \end{bmatrix}\quad
\begin{Bmatrix} a & b \\ c & d\\ \end{Bmatrix}\quad
\begin{vmatrix} a & b \\ c & d \\ \end{vmatrix}\quad
\begin{Vmatrix} a & b\\ c & d \\ \end{Vmatrix}
$$

长公式

align	最基本的对齐环境
multline	非对齐环境
gather	无对齐的连续方程
$$
\begin{align}
	x = a+b+c+{} \\
   		 d+e+f+g
  \end{align}
$$

$$
\begin{multline}
	x = a+b+c+{} \\
   		 d+e+f+g
  \end{multline}
$$

$$
\begin{split}
x = & a + b + c +\\
	& d + e + f + g
\end{split}
$$

$$
\begin{gather}
a = b+c+d\\
x=y+z
\end{gather}
$$

$$
\begin{align}
a &=b+c+d \\
x &=y+z
\end{align}
$$

分支公式
分段函数通常用cases次环境携程分支公式:
$$
y=\begin{cases}
-x,\quad x\leq 0\\
x, \quad x>0
\end{cases}
$$

$$
e^{i\theta}=cos\theta+\sin\theta i\tag{1}
$$

$$
e^{i\pi}+1=0
$$

矩阵，\\\是换行，正文里两个\\后面跟字符，\会被吃掉一个；
$$
bmatrtix\begin{bmatrix}
1 & 2 & 3 \\
4 & 5 & 6 \\
7 & 8 & 9 \\
\end{bmatrix} \tag{1}
$$

$$
left[right]\left[
\begin{matrix}
1 & 2 & 3 \\
4 & 5 & 6 \\
7 & 8 & \alpha \\
\end{matrix} \right]
\tag{2}
$$

各种括号，中括号的矩阵 {bmatrix}
$$
\begin{matrix}{121}\end{matrix} \\
\begin{pmatrix}{121}\end{pmatrix} \\
\begin{bmatrix}{121}\end{bmatrix} \\
\begin{Bmatrix}{121}\end{Bmatrix} \\
\begin{vmatrix}{121}\end{vmatrix} \\
\begin{Vmatrix}{121}\end{Vmatrix} \\
$$
函数符号
$$
sin\\
\ln(x)\\
log2^{10}\\
\log_2{10}\\
$$
文件编码直接支持的符号
$$
±	\pm 
×	\times
÷	\div\\
∑	\sum  ∏	\prod\\
≠	\neq
≤	\leq
≥	\geq
$$

$$
大括号: {a+x}	\lbrace a+x \rbrace\\
尖括号: ⟨x⟩	\langle x \rangle\\
上取整: ⌈x/2⌉	\lceil \frac{x}{2} \rceil\\
下取整: ⌊x⌋	\lfloor x \rfloor\\
$$

$$
\lbrace \sum_{i=0}^{n}i^{2}=\frac{2a}{x^2+1} \rbrace\\
\left\lbrace \sum_{i=0}^{n}i^{2}=\frac{2a}{x^2+1} \right\rbrace
$$





---

[引用](https://www.jianshu.com/p/25f0139637b7)

行外公式
$$
\Gamma(z) = \int_0^\infty t^{z-1}e^{-t}dt\,.
$$

$$ {11}
行内公式
\Gamma(z) = \int_0^\infty t^{z-1}e^{-t}dt\,.
$$ {21}

$$
基本的上下标操作，空格无效——x_i^2
$$

括号
小括号与方括号
  使用原始的( ) ，[ ] 即可，如
$$
(2+3)[4+4]{中括号}{}
$$

  使用\left(或\right)使符号大小与邻近的公式相适应（该语句适用于所有括号类型），如
$$
\left(\frac{x}{y}\right)
$$
分式
$$
\frac{1}{1+\frac{1}{2}}
$$

$$
(\frac{a+c+1}{b+c+2})
$$

根号，可用空格表示√
$$
\sqrt{2}<\sqrt[3]{3}
$$

$$
\sqrt[3]{\frac xy}
$$


### 分类表达式

  定义函数的时候经常需要分情况给出表达式，使用`\begin{cases}…\end{cases}` 。其中：

-   使用`\\` 来分类，
-   使用`&` 指示需要对齐的位置，
-   使用`\` +`空格`表示空格。


$$
f(n)
\begin{cases}
\cfrac n2, &if\ n\ is\ even\\
3n + 1, &if\  n\ is\ odd
\end{cases}
$$

$$
x^{y^z}=(1+{\rm e}^x)^{-2xy^w}
$$

LaTeX Equation

Example: $x^{y^z}=(1+{\rm e}^x)^{-2xy^w}$

typora有一个自动添加回车问题，会导致$$之后多回车，以至于渲染公式失败；


$$
\sum_{i=0}^{n}i^2
$$
$\sum{i=0}^{n}i^2$    %第一种
$$\sum{i=0}^{n}i^2$$    %第二种
$$
\sum_{i=0}^{n}i^2    %第一种
%注释不显示，默认无回车
%\sum_{i=0}^{n}i^2    %第二种
$$
公式语法，$使用略有不同


$$
\sum_{i=0}^{n}i^2_1 % ^ _顺序无关
$$

$$
\\紧贴 $a\!b$
\\没有空格 $ab$
\\小空格 a\,b
\\中等空格 a\;b
\\大空格 a\ b
\\quad空格 $a\quad b$
\\两个quad空格 $a\qquad b$
$$



各种顶部符号

\\$ \bar{x} $=>x¯ $ \acute{x}$=>x´ $ \mathring{x}$=>x˚
$$
\\$ \vec{x}$=>x⃗  $ \grave{x} $=>x` $ \dot{x}$=>x˙
\\$ \hat{x}$=> x^ $ \tilde{x}$=>x~ $ \ddot{x}$=>x¨
\\$ \check{x} $=>xˇ $ \breve{x}$=>x˘ $ \dddot{x} $=>x...
$$
各种顶部符号
$$
\bar{x} =>x¯  \acute{x}=>x´  \mathring{x}=>x˚ \\
 \vec{x}   \grave{x}  \dot{x}  \\
 \hat{x}   \tilde{x}  \ddot{x} \\
 \check{x}  \breve{x} \dddot{x}
$$
$$
$$\int_a^b f(x)\mathrm{d}x$$
插入小空格：$$\int_a^b f(x)\,\mathrm{d}x$$

$$\left(\sum_{k=\frac{1}{2}}^{N^2}\frac{1}{k}\right)$$
$$

矩阵
$$
\\$$\begin{matrix}1 & 2\\3 &4\end{matrix}$$
\\$$\begin{pmatrix}1 & 2\\3 &4\end{pmatrix}$$
\\$$\begin{bmatrix}1 & 2\\3 &4\end{bmatrix}$$
\\$$\begin{Bmatrix}1 & 2\\3 &4\end{Bmatrix}$$
\\$$\begin{vmatrix}1 & 2\\3 &4\end{vmatrix}$$
\\$$\begin{Vmatrix}1 & 2\\3 &4\end{Vmatrix}$$
$$

$$
\mathbf{X} =
\left( \begin{array}{ccc}
x_{11} & x_{12} & \ldots \\
x_{21} & x_{22} & \ldots \\
\vdots & \vdots & \ddots
\end{array} \right)
$$

$$
y = 
\left\{
\begin{array}{}
a & \textrm{if $d^2>4ac$}\\
b+x & \textrm{in the morning}\\
l & \textrm{all day long}
\end{array}
\right.
$$

$$
\left(\begin{array}{c|c}
1 & 2 \\
\hline
3 & 4
\end{array}\right)
$$

---

## **希腊字母**

|  名称   | 大写 |  code   |   小写   |   code   |
| :-----: | :--: | :-----: | :------: | :------: |
|  alpha  |  A   |    A    |    α     |  \alpha  |
|  beta   |  B   |    B    |    β     |  \beta   |
|  gamma  |  Γ   | \Gamma  |    γ     |  \gamma  |
|  delta  |  Δ   | \Delta  |    δ     |  \delta  |
| epsilon |  E   |    E    |    ϵ     | \epsilon |
|  zeta   |  Z   |    Z    |    ζ     |  \zeta   |
|   eta   |  H   |    H    |    η     |   \eta   |
|  theta  |  Θ   | \Theta  |    θ     |  \theta  |
|  iota   |  I   |    I    |    ι     |  \iota   |
|  kappa  |  K   |    K    |    κ     |  \kappa  |
| lambda  |  Λ   | \Lambda |    λ     | \lambda  |
|   mu    |  M   |    M    |    μ     |   \mu    |
|   nu    |  N   |    N    |    ν     |   \nu    |
|   xi    |  Ξ   |   \Xi   |    ξ     |   \xi    |
| omicron |  O   |    O    |    ο     | \omicron |
|   pi    |  Π   |   \Pi   |    π     |   \pi    |
|   rho   |  P   |    P    |    ρ     |   \rho   |
|  sigma  |  Σ   | \Sigma  |    σ     |  \sigma  |
|   tau   |  T   |    T    |    τ     |   \tau   |
| upsilon |  Υ   |    υ    | \upsilon |          |
|   phi   |  Φ   |  \Phi   |    ϕ     |   \phi   |
|   chi   |  X   |    X    |    χ     |   \chi   |
|   psi   |  Ψ   |  \Psi   |    ψ     |   \psi   |
|  omega  |  Ω   | \Omega  |    ω     |  \omega  |



---

## LaTeX

富文本和标记语言
富文本存储了格式等信息（document.xml,）ml就是标记语言（Markup Language）的缩写。
富文本编辑器 Rich Text Editor；所见即所得的方式排版；[在线](http://www.wangeditor.com/)
标记语言，是一种将文本以及文本相关的其他信息结合起来，展现出关于文档结构和数据处理细节的电脑文字编码。
Markdown 是一种用来写作的轻量级「标记语言」，它用简洁的语法代替排版，而不像一般我们用的字处理软件 Word 或 Pages 有大量的排版、字体设置。

LaTeX，始于公式，忠于优雅
LaTeX简单来说就是一种文字处理软件/计算机标记语言。广义上的计算机标记语言（比如HTML），通过一些简单的代码表达出精确的含义，具有不二义性。
通过简单的语法写出优雅高贵的数学公式，目前Markdown已经支持LaTeX语法的公式
LATEX 是一种排版系统，它非常适用于生成高印刷质量的科技和数学类文档。这个系统同样适用于生成从简单信件到完整书籍的所有其他种类的文档。LATEX 使用 TEX作为它的格式化引擎。

TEX 是一个以排版文章及数学公式为目标的计算机程序。由Pascal 语言写成，特点: 免费、输出质量高、擅长科技排版、有点像编程。

[quote](https://www.cnblogs.com/sddai/p/12046242.html)

CTEX 国内致力于TEX 推广的网站：[科技排版系统](http://www.ctex.org/ )

Tex学习成本太高，只学习数学公式语法，与markdown基本通用；

**基础知识**

1.LATEX控制序列的概念（类似于函数）

控制序列可以是作为命令：以“\”开头，参数：必须参数{}和可选参数[]。

2.环境概念

以“bengin 环境名”开始，并以“end 环境名”结束。

3.LATEX可以排版公式与文字，故分为：数学模式和文本模式。如果你想要在公式中排版普通的文本（直立字体和普通字距），那么你必须要把这些文本放在\textrm{...} 命令中。

4.在数学模式中又分为两种，一种是公式排版在一个段落之中；另一种是公式独立形式排版。前一种，公式直接放在文字之间，公式高度一般受文本高度限制；后一种，公式另起一行，高度可调整。处于段内的数学文本要放在\\( 与\\) 之间，或者\\begin{displaymath} 与\\end{displaymath} 之间（为了网页显示，这里用双斜杠表示单斜杠）

**数学公式语法**

行内公式与行外公式，直接在公式前加文字；

\1. 上标与下标

上标命令是 ^{角标}，下标命令是 _{角标}。当角标是单个字符时可以不用花括号（在 LaTeX 中，花括号是用于分组，即花括号内部文本为一组）。

2.分式 

可用命令：\frac{分子}{分母}。

3.根式

排版根式的命令是：开平方：\sqrt{表达式}；开 n 次方：\sqrt[n]{表达式}

4. 求和与积分

排版求和符号与积分符号的命令分别为 \sum 和 \int，它们通常都有上下限，在排版上就是上标和下标。



---

