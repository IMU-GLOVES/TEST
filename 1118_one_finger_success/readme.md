## arduino
- 從傳四元數改成傳三軸六個資訊，所以傳了18個data。

## python
- 將 madgwick 演算法，移到 python 內，減少 node-mcu上的計算負擔。 
- 目前可以模擬手指直立的情況，但還缺一些 dof 關節角度限制。