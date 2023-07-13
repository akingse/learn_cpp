



```mermaid
graph TD
A(开始) --> B[初始化]
    B --> C{INT == 0}
    C --> |a=1| D[结果1]
   



```

### 语法

定义框的名称和类型

st=>start: 开始框
op=>operation: 处理框
cond=>condition: 判断框(是或否?)
sub1=>subroutine: 子流程
io=>inputoutput: 输入输出框

指定流程关系

st->op->cond
cond(yes)->io->e
cond(no)->sub1(right)->e

```flow
st=>start: 开始框
op=>operation: 处理框
cond=>condition: 判断框(是或否?)
sub1=>subroutine: 子流程
io=>inputoutput: 输入输出框
e=>end: 结束框
st->op->cond
cond(yes)->io->e
cond(no)->sub1(right)->e
```