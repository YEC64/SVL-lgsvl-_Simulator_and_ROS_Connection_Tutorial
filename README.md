# SVL(LGSVL) SIMULATOR

# SVL SIMULATOR Nedir ?

SVL, LG ve UNITY desteği ile ortaya çıkmış bir araç simülasyon programıdır. SVL ile kolayca gerçek zamanlı bir trafik simülasyonu başlatabilirsiniz. ROS  veya API bağlantısı ile araç üzerindeki sensörlerden veri alınabilir ve araca haraket verilebilir. Böylece SVL kullanıcılara otonom sürüş, görüntü işleme, sensör füzyon vebenzeri birçok algoritmayı trafikteki bir araç üzerinde denemeye olanak sağlar. 

Bu repoda sizlere SVL simülasyon programının indirilmesini, yeni bir ros çalışma alanı oluşturarak nasıl ros bağlantısını gerçekleştireceğinizi ve ros bağlantısı üzerinden nasıl python kodlarınızı araç üzerinde deneyebileceğinizi adım adım aktaracağım.

NOT: Repo Ubuntu 20.04 işletim sistemi üzerinde, Ros Noetic ve SVL’in yeni sürümüne kıyasla çok daha kullanışlı olduğunu düşündüğüm 2020.06 sürümüne göre oluşturulmuştur.

  

# Simülasyon Programının İndirilmesi

Aşağıdaki link üzerinden SVL simülatörün 2020.06 linux versiyonunu indirebilirsiniz. 

link: [https://github.com/lgsvl/simulator/releases](https://github.com/lgsvl/simulator/releases)

İndirme işleminin ardından zipten çıkardıktan sonra “simulator” dosyasını çalıştırarak doğrudan simülasyonu çalıştırabilirsiniz, herhangi bir kurulum işlemi gerektirmez. 

NOT: Çalıştırdığınızda bir değişiklik olmuyorsa muhtemelen ubuntu işletim sisteminiz için varsayılan ekran kartının, işlemciler için harici ekran kartı olmaması durumunda kullanılabilmesi için konulan dahili ekran kartınız olmasıdır. Bu durum simülasyonun hiç çalışmamasına ya da yavaş ve gecikmeli çalışmasına sebep olur. Bu sebeple harici ekran kartınızı varsayılan olarak ayarlamanınız gerekmektedir.

### Varsayılan ekran kartını kontrol etme ve değiştirme

Ubuntuda “ayarlara” giderek “hakkında” kısmına girin  “Graphics” kısmında işletim sisteminizin kullandığı ekran ekrankartını görebilirsiniz.

![Screenshot from 2022-09-07 02-26-10.png](SVL(LGSVL)%20SIMULATOR%20aa0cb6bdf0034bf7a913a6b913c0b5ef/Screenshot_from_2022-09-07_02-26-10.png)

Eğer harici ekran kartınız yerine dahili ekran kartınızın ismi yazıyorsa ekran kartınıza uygun driver yazılımını kurmanız gerekmekte.

Nvidia ekran kartınız varsa driver kurulu olup olmadığını kontrol etmek için terminalde aşağıdaki komutu çalıştırın

```bash
nvidia-smi
```

Çıktı aşağıdaki görseldekine benzer ise driver yazılımı kuruludur değilse kurmak gerekmektedir.

![Screenshot from 2022-09-07 03-02-18.png](SVL(LGSVL)%20SIMULATOR%20aa0cb6bdf0034bf7a913a6b913c0b5ef/Screenshot_from_2022-09-07_03-02-18.png)

Nvidia ekran kartını, uygun driver yazılımını kurduktan sonra, varsayılan yapmak için terminalde aşağıdaki komutu çalıştırın ve bilgisayarınızı yeniden başlatın.

```bash
sudo prime-select nvidia
```

# Simülasyonun Çalıştırılması

İndirdiğiniz dosyalardan ‘simulator’ dosyasını çalıştırarak simülasyon programını açın.Aşağıdaki ekran sizi karşılayacak 

![Screenshot from 2022-09-07 03-14-33.png](SVL(LGSVL)%20SIMULATOR%20aa0cb6bdf0034bf7a913a6b913c0b5ef/Screenshot_from_2022-09-07_03-14-33.png)

“Open Browser” ‘a tıkladığınızda tarayıcınızda açılan sayfa üzerinden simülasyon ile ilgili ayarları yapabilirsiniz 

![Screenshot from 2022-09-07 03-17-58.png](SVL(LGSVL)%20SIMULATOR%20aa0cb6bdf0034bf7a913a6b913c0b5ef/Screenshot_from_2022-09-07_03-17-58.png)

“Simulations” kısmından birtanesini seçerek sağ altta yer alan play tuşundan başlatmanız yeterli, simülasyonunuz çalışmaya başlayacaktır.

![Screenshot from 2022-09-07 03-21-08.png](SVL(LGSVL)%20SIMULATOR%20aa0cb6bdf0034bf7a913a6b913c0b5ef/Screenshot_from_2022-09-07_03-21-08.png)

NOT: Aracı ROS ile bağlamak istediğimiz için biz “(With Autoware)” olan simülasyon ortamını kullanacağız.

Simülasyon ortamı ile ilgili değişikleri simülasyon isimlerinin yanında yer alan butona bastığınızda açılacak olan edit ekranından gerçekleştirebilirsiniz.(araç ve harita değişikliği gibi)

# ROS Bağlantısının Gerçekleştirilmesi

Önce kullanacağımız ROS dağıtımının kurulması gerekmektedir. Aşağıdaki linke tıklayarak ROS Noetic sürümünü kurabilirsiniz.

ROS Noetic:  [http://wiki.ros.org/noetic/Installation/Ubuntu](http://wiki.ros.org/noetic/Installation/Ubuntu)

## ROS Çalışma Alanı Oluşturma

Çalışma alanı için klasör oluşturma:

```bash
mkdir -p ./catkin_ws/src
```

Çalışma alanını derleme:

```bash
cd catkin_ws
catkin_make
```

Çalışacapımız ros alanını kaynak gösterme:

```bash
source /home/catkin_ws/devel/setup.bash
```

## ROS Çalışma alanına gerekli paketleri oluşturma

Gerekli dizine gitme:

```bash
cd catkin_ws/src
```

Paket oluşturma:

```bash
catkin_create_pkg  paket_adi  rospy roscpp
```

Çalışma alanını derleme:

```bash
cd /catkin_ws
catkin_make
```

## ROS - Simülasyon bağlantısı

Bağlantının gerçekleştirilebilmesi için ROS’ un diğer yazılımlarla haberleşmesini sağlayan “bridge” yapısının indirilmesi gerekmektedir.

```bash
sudo apt-get install ros-noetic-rosbridge-suite
```

Ros ve simülasyon ortamını haberleştirmek için gerekli kurulumları yaptık. Tek yapmanız gereken aşağıdaki adımlar izlemek

### 1. Simülasyonu başlat

![Screenshot from 2022-09-08 13-46-06.png](SVL(LGSVL)%20SIMULATOR%20aa0cb6bdf0034bf7a913a6b913c0b5ef/Screenshot_from_2022-09-08_13-46-06.png)

Asağıda yer alan fiş sembolüne tıkladığınızda bridge statüs kısmının “Disconnected” olduğunu görebilirsiniz.

### 2. Üzerinde çalışacağımız ros çalışma alanını kaynak göster

```bash
source /home/catkin_ws/devel/setup.bash
```

### 3. Bridge yapısını ve ROS’u başlat

```bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

![Screenshot from 2022-09-08 13-49-40.png](SVL(LGSVL)%20SIMULATOR%20aa0cb6bdf0034bf7a913a6b913c0b5ef/Screenshot_from_2022-09-08_13-49-40.png)

Komutu çalıştırdıktan sonra bridge status kısmının “Connected” olduğunu görebilirsiniz.

## Terminal üzerinden araç verilerinin alınması

Bir önceki sımında yaptığımız ros bağlantısı çalışıyorken, yeni bir terminal açıp aşağıdaki komutu çalıştırarak araç ile ilgili rostopic’leri görebilirsiniz

```bash
rostopic list
```

Bu topiclerden yayımlanan verileri terminale bastırmak için aşağıdaki komutu çalıştırmanız yeterli

```bash
rostopic echo /TopicName
```

## Python kodları üzerinden Ros  verilerinin alınması

Python ile ros topiclerinden veri alabilmemiz için ros subscriber tanımalamamız gerekmektedir.Ros işle ilgili kaynaklardan kolayca öğrenebilirsiniz. 

Lidar verisi için örnek:

```python
import sys
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

class Lidar_Feature:
    def __init__(self):
        
        self.subscriber2 = rospy.Subscriber("/points_raw", PointCloud2, self.callback_lidar, queue_size = 10)
        
        rospy.init_node("ays", anonymous = True)
        rate = rospy.Rate(10)   #10Hz
   

    def callback_lidar (self, ros_data):

        print(ros_data)

def main(srgs):
    lf = Lidar_Feature()
    try:
        rospy.spin()
        
    except KeyboardInterrupt:
        print("Ros Shutdown")

main(sys.argv)
```

Benzer yapı ile ekleyeceğiniz veya halihazırda var olan sensörlerin verilerine ulaşabilirsiniz.

### Python kodları ile aracın hareket ettirilmesi

Araca hareket verebilmemiz için uygun mesaj tipinde veri yayınlamamız gerekmektedir. Bunun için Ros publisher oluşturmak gerekiyor.

Örnek kod:

```python
import sys
import numpy as np
import rospy
from autoware_msgs.msg import VehicleCmd  
import time

class ilerleme_feature:
    
    
    def __init__(self):
        self.velocity = 10
        self.angle = 0.5
        self.msg = VehicleCmd()
        self.pub = rospy.Publisher('/vehicle_cmd',VehicleCmd, queue_size = 10)
        
        rospy.init_node('hareket',anonymous=True)
        rate = rospy.Rate(10) # 10hz

        self.msg.twist_cmd.twist.angular.z = self.angle
        self.msg.twist_cmd.twist.linear.x = self.velocity
        self.pub.publish(self.msg)

       

def main(args):

    lf = ilerleme_feature()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutdown")

main(sys.argv)
```

# Kapanış

Selam ben YEC. Elimden geldiğince anlaşılır bir kaynak oluşturmaya çalıştım, umarım faydalı olmuştur. Simülasyon ortamına Unity üzerinden düzenlediğiniz farklı harita ve araçları kolayca simülasyon ortamına entegre edebilirsiniz. Detaylarını dökümantasyonlarda bulabilirsiniz.

ROS Dökümantasyon: [http://wiki.ros.org/Documentation](http://wiki.ros.org/Documentation)

SVL Dökümantasyon:  [https://www.svlsimulator.com/docs/](https://www.svlsimulator.com/docs/)

Sorularınız olursa Linkedin üzerinden sorabilirsiniz. İyi çalışmalar 🙂

Linkedin: [https://www.linkedin.com/in/yemrecoskun/](https://www.linkedin.com/in/yemrecoskun/)