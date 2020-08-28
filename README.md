Sử dụng ORTools trong NS3
1. Tải và giải nén ORTools

Phiên bản ORTools sử dụng là 7.6.7691, lựa chọn phiên bản ứng với Ubuntu đang sử dụng:
	https://github.com/google/or-tools/releases/tag/v7.6

Nếu sử dụng phiên bản Ubuntu 16 thì tải file or-tools_ubuntu-16.04_v7.6.7691.tar.gz

2. Copy thư viện và header của ORTOOLS
	- Chạy lệnh sau để tìm thư mục ưu tiên khi tìm kiếm library của gcc:
	
		gcc -print-search-dirs | sed '/^lib/b 1;d;:1;s,/[^/.][^/]*/\.\./,/,;t 1;s,:[^=]*=,:;,;s,;,;  ,g'
	- Kết quả trả về nhiều thư mục khác nhau, tìm thư mục đầu tiên, giả sử thư mục đó là "/usr/lib/gcc/x86_64-linux-gnu/5/" thì thực hiện lệnh sau để copy thư viện của ORTools:
	
		cd thư-mục-github

		cd ORTools
		
		sudo cp lib/* /usr/lib/gcc/x86_64-linux-gnu/5/
	
		sudo cp -rf include/* /usr/local/include

3. Tải phiên bản ns-3.27 tại: https://www.nsnam.org/releases/ns-3-27/download/

4. Giải nén file tải về, thư mục chứa waf có đường dẫn là: thư-mục-tải-ns3/ns-allinone-3.27/ns-3.27

5. Thay wscript của module core và thư mục chứa waf

	cd thư-mục-github
	
	cp core_wscript/wscript thư-mục-chứa-waf/src/core
	
	cp waf_wscript/wscript thư-mục-chứa-waf
6. Cài đặt các gói cần thiết và biên dịch NS3

	sudo apt-get install gcc g++ python python3
	
	cd thư-mục-chứa-waf
	
	./waf configure --disable-tests --disable-examples --disable-python
	
	./waf
	
7. Mô tả kịch bản và tham số mô phỏng: https://sites.google.com/site/teamemnetlab411/project-updates-1/ketquamophongsmartcity


8. Các file trong thư mục CodeNs3
- Các trường hợp có chung 5 file là: "macro_param.h", "communication.h", "misc.h", "scenario.h" và "handle.h"
- Mỗi trường hợp khác nhau ở 2 file là: "tên-trường-hợp.cc" và "kinematic_tên-trường-hợp.h"
- Trường hợp CVRP và Intercell có thêm file "vrp_capacity.h" để giải bài toán CVRP
- Trong file macro_param.h, ta sẽ thay đổi 2 tham số:
	+ TOTAL_SITE: từ 300 đến 700
	+ MAX_RESOURCE_PER_UAV: 250 đến 600 
9. Thư mục Matlab_Excel

File excel là thống kê kết quả chạy NS-3, sau đó bảng số liệu được dùng để vẽ đồ thị trong Matlab