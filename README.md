Sử dụng ORTools trong NS3
1. Tải binary package của ORTools tại: https://developers.google.com/optimization/install/cpp/linux và cài đặt build tools cho ORTools
	sudo apt-get -y install build-essential zlib1g-dev
2. Giải nén file tải về
3. Copy thư viện và header của ORTOOLS
	- Chạy lệnh sau để tìm thư mục ưu tiên khi tìm kiếm library của gcc:
		gcc -print-search-dirs | sed '/^lib/b 1;d;:1;s,/[^/.][^/]*/\.\./,/,;t 1;s,:[^=]*=,:;,;s,;,;  ,g' | tr \; \\012
	- Kết quả trả về nhiều thư mục khác nhau, tìm thư mục đầu tiên, giả sử thư mục đó là "/usr/lib/gcc/x86_64-linux-gnu/5/" thì thực hiện lệnh sau để copy thư viện của ORTools:
		cd thư-mục-giải-nén-ortools
		sudo cp lib/* /usr/lib/gcc/x86_64-linux-gnu/5/
	- Copy header của ORTools:
		sudo cp -rf include/* /usr/local/include
4. Tải phiên bản ns-3.27 tại: https://www.nsnam.org/releases/ns-3-27/download/
5. Giải nén file tải về, thư mục chứa waf có đường dẫn là: thư-mục-tải-ns3/ns-allinone-3.27/ns-3.27
6. Thay wscript của module core và thư mục chứa waf
	cd thư-mục-github
	cp core_wscript/wscript thư-mục-chứa-waf/src/core
	cp waf_wscript/wscript thư-mục-chứa-waf
7. Cài đặt các gói cần thiết và biên dịch NS3
	sudo apt-get install gcc g++ python python3
	cd thư-mục-chứa-waf
	./waf configure --disable-tests --disable-examples --disable-python
	./waf
Các file trong thư mục CodeNs3
- Các trường hợp có chung 5 file là: "macro_param.h", "communication.h", "misc.h", "scenario.h" và "handle.h"
- Mỗi trường hợp khác nhau ở 2 file là: "tên-trường-hợp.cc" và "kinematic_tên-trường-hợp.h"
- Trường hợp CVRP và Intercell có thêm file "vrp_capacity.h" để giải bài toán CVRP