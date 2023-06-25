```C++
//box judge
((bounding.min() - bounding.max() - tolerance_longlong).array() > 0).any()
//判出isSepa
	box12.min() = box12.min() - tolerance_double;
	box12.max() = box12.max() + tolerance_double;
	bool isSepa0 = !box12.intersects(box22);

cwiseQuotient //对矩阵或数组的每个元素执行除法操作
// intersects判定
(a.min.array()<=(b.max)().array()).all() && ((b.min)().array()<=a.max.array()).all();
//box intersection规则
min=MaxBox.min()
min=MinBox.max()
    
bool is_soft = tolerance > 0 ? ((bounding.min() - bounding.max()).array() > 0).any() : false;

if (tolerance > 0) //tolerance > 0 软碰撞
    return !box1.intersects(box2); 
	isSape == true // boxSepa, enlarge boxInter ==> getDist, if d>tole exclude
	isInter==false // boxInter ==>if isInter, true: d=0, false: getDist
else
    return false; //tolerance==0 硬碰撞，boxInter ==> only judge isInter



tolerance == 0
	MeshIntrusionTesting

tolerance > 0
    isInter==false //boxInter
    	if MeshIntrusionTesting
            d=0
        else
            MeshStandoffDistance
    isSape == true //boxSepa
    	MeshStandoffDistance
```

