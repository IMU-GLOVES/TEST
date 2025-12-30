### 操作注意事項:

每次新加入一個IMU就要改的

##### Arduino code要改的:

---要確定6050\_and\_9250\_data的DEBUG\_MODE =  false

---6050\_and\_9250\_data要在setup()設定好現在有幾顆IMU，每個channel都要對應好(例:imus\[0] = new MPU6050\_Node(MUX\_ADDR\_A, 0); // 食指指尖 (接 SD0))，預設最後一顆都是九軸的channel，所以硬體上最好真的讓九軸的在多工器上的最後一個channel

##### Python code要改的:

---visualizer\_class\_version.py 要改的都在主程式區塊

---ctrl+f 找 my\_setup = \[6] \* (六軸數量) + \[9]

---增加了哪一個指節的IMU就要新增那個指節的物件，可以直接複製一個區塊去改

---例如複製plam = HandSegment()這整段，plam這個變數本身就要改

---現在的命名邏輯是拇指是index1，食指是index2，以此類推。指根指節是base，指中指節是mid，指尖指節是top， 所以以食指指尖為例就是index2\_top。

---這個HandSegment()裡面要改的如下

---name是給人看的，我現在都跟變數名稱取一樣

---parent要改成這個指節跟隨的物件，每個指根指節(base)的parent都是手掌(palm)，每個指中指節(mid)的parent都是指根指節(base)，每個指尖指節(top)的parent都是指中指節(mid)。以中指指中指節(index3\_mid)為例，他的parent是中指指根指節(index3\_base)

---pos_offset是這個指節的起點，只有每個指根指節需要調這個相對於手掌的座標就好，其他的都是跟著上一節動

---imu\_index，這個要根據資料從imu9_serial_manager出來的排序，指節如果是連接第一個多工器的channel 0~7的index就是0~7，如果是連接第二個多工器的channel 就是8~13(總共只會用到14個六軸所以是0~13，掌心我已經設定一定是最後一個了，現在也接在第二個多工器的channel 7了)

---可以的話把註解也都完整複製跟修改，比較清楚這是哪根手指頭的code，註解的\[索引]、ch\*也要記得改
---最後記得要把新增的指節變數加入到hand_parts這個list



### 

### 真的有用到的:

requirements.txt

##### **Arduino:**

##### **6050\_and\_9250\_data(硬體控制層)**

負責跟NodeMCU下指令，現在NodeMCU只負責收二進制數據傳到電腦裡，給電腦端做計算這樣最快。

code開頭的DEBUG\_MODE可以切換debug mode or binary mopde



##### **Python:**

##### **imu9\_serial\_manager.py(數據切分層)**

收數據、切分數據，通常不用改也不用執行這個。

##### **imu9\_data\_to\_math.py(數學層)**

把接收到的數據(加速度、角速度)轉成可以用的角度數據(roll、yaw、pitch)

##### **visualizer\_class\_version.py(視覺化層)**

把數據轉成視覺化。



### 測試用的:

##### only\_6050\_data

##### serial\_manager.py

##### data\_to\_math.py

##### visualizer.py

##### visualizer\_test.py

##### 

