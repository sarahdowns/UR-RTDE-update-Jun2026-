import pycurl
import xmlrpc.client
from io import BytesIO


class VG10:
    def __init__(self, robot_ip, vg_id):
        self.vg_id = vg_id
        self.robot_ip = robot_ip

    def vg10_release(self, channelA: bool = True, channelB: bool = True):
        
        self.channelA = channelA # True/False
        self.channelB = channelB # True/False
  
        xml_request = f"""<?xml version="1.0"?>
        <methodCall>
        <methodName>vg10_release</methodName>
            <params>
                    <param>
                    <value><int>{self.vg_id}</int></value>
                    </param>
                    <param>
                    <value><boolean>{self.channelA}</boolean></value>
                    </param>
                    <param>
                    <value><boolean>{self.channelB}</boolean></value>
                    </param>
            </params>
        </methodCall>"""

        headers = ["Content-Type: application/x-www-form-urlencoded"]

        # headers = ["User-Agent: Python-PycURL", "Accept: application/json"]
        data = xml_request.replace('\r\n','').encode()
        # Create a new cURL object
        curl = pycurl.Curl()

        # Set the URL to fetch
        curl.setopt(curl.URL, f'http://{self.robot_ip}:41414')
        curl.setopt(curl.HTTPHEADER, headers)
        curl.setopt(curl.POSTFIELDS, data)
        # Create a BytesIO object to store the response
        buffer = BytesIO()
        curl.setopt(curl.WRITEDATA, buffer)

        # Perform the request
        curl.perform()

        # Get the response body
        response = buffer.getvalue()

        # Print the response
        print(response.decode('utf-8'))

        # Close the cURL object
        curl.close()

    def vg10_grip(self, channel, vacuum_percent):
        
        self.channel = channel # 0,1,2
        self.vacuum_percent = vacuum_percent # softgrip => 30 = 30%, firm grip => 60 = 60%

        xml_request = f"""<?xml version="1.0"?>
        <methodCall>
        <methodName>vg10_grip</methodName>
            <params>
                <param>
                    <value><int>{self.vg_id}</int></value>
                </param>
                <param>
                    <value><int>{self.channel}</int></value>
                </param>
                <param>
                    <value><double>{self.vacuum_percent}</double></value>
                </param>
            </params>
        </methodCall>
        """

        headers = ["Content-Type: application/x-www-form-urlencoded"]

        # headers = ["User-Agent: Python-PycURL", "Accept: application/json"]
        data = xml_request.replace('\r\n','').encode()
        # Create a new cURL object
        curl = pycurl.Curl()

        # Set the URL to fetch
        curl.setopt(curl.URL, f'http://{self.robot_ip}:41414')
        curl.setopt(curl.HTTPHEADER, headers)
        curl.setopt(curl.POSTFIELDS, data)
        # Create a BytesIO object to store the response
        buffer = BytesIO()
        curl.setopt(curl.WRITEDATA, buffer)

        # Perform the request
        curl.perform()

        # Get the response body
        response = buffer.getvalue()

        # Print the response
        print(response.decode('utf-8'))

        # Close the cURL object
        curl.close()


class RG2:
    def __init__(self, robot_ip, rg_id):
        self.rg_id = rg_id
        self.robot_ip = robot_ip

    def get_rg_width(self):
        xml_request = f"""<?xml version="1.0"?>
    <methodCall>
        <methodName>rg_get_width</methodName>
            <params>
                <param>
                    <value><int>{self.rg_id}</int></value>
                </param>
            </params>
    </methodCall>"""

        headers = ["Content-Type: application/x-www-form-urlencoded"]

        data = xml_request.replace('\r\n','').encode()

        # Create a new cURL object
        curl = pycurl.Curl()

        # Set the URL to fetch
        curl.setopt(curl.URL, f'http://{self.robot_ip}:41414')
        curl.setopt(curl.HTTPHEADER, headers)
        curl.setopt(curl.POSTFIELDS, data)
        # Create a BytesIO object to store the response
        buffer = BytesIO()
        curl.setopt(curl.WRITEDATA, buffer)

        # Perform the request
        curl.perform()

        # Get the response body
        response = buffer.getvalue()

        # Print the response
        print(response.decode('utf-8'))

        # Close the cURL object
        curl.close()

        xml_response = xmlrpc.client.loads(response.decode('utf-8'))
        rg_width = float(xml_response[0][0])
        #print(rg_width)
        return rg_width


    def rg_grip(self, target_width: float = 100, target_force: float= 10) -> bool:
        #assert target_width <= 100 and target_width >= 0, 'Target Width must be within the range [0,100]'
        #assert target_force <= 40 or target_force >= 0, 'Target force must be within the range [0,40]'

        # WARNING: params will be sent straight to electrical system with no error checking on robot!
        # if (target_width > 100):
        #     target_width = 100
        # if(target_width < 0):
        #     target_width = 0
        # if(target_force > 40):
        #     target_force = 40
        # if(target_force < 0):
        #     target_force = 0

        xml_request = f"""<?xml version="1.0"?>
    <methodCall>
    <methodName>rg_grip</methodName>
        <params>
            <param>
                <value><int>{self.rg_id}</int></value>
            </param>
            <param>
                <value><double>{target_width}</double></value>
            </param>
            <param>
                <value><double>{target_force}</double></value>
            </param>
        </params>
    </methodCall>"""

        headers = ["Content-Type: application/x-www-form-urlencoded"]

        # headers = ["User-Agent: Python-PycURL", "Accept: application/json"]
        data = xml_request.replace('\r\n','').encode()
        # Create a new cURL object
        curl = pycurl.Curl()

        # Set the URL to fetch
        curl.setopt(curl.URL, f'http://{self.robot_ip}:41414')
        curl.setopt(curl.HTTPHEADER, headers)
        curl.setopt(curl.POSTFIELDS, data)
        # Create a BytesIO object to store the response
        buffer = BytesIO()
        curl.setopt(curl.WRITEDATA, buffer)

        # Perform the request
        curl.perform()

        # Get the response body
        response = buffer.getvalue()

        # Print the response
        print(response.decode('utf-8'))

        # Close the cURL object
        curl.close()

    def rg_open(self, target_force: float = 40.0):
        """Opens the gripper to maximum width."""
        self.rg_grip(target_width=100.0, target_force=target_force)

    def rg_close(self, target_force: float = 40.0):
        """Closes the gripper to minimum width."""
        self.rg_grip(target_width=0.0, target_force=target_force)


class TwoFG7():
    def __init__(self, robot_ip: str, id: int):
        self.robot_ip = robot_ip
        self.id = id

        self.max_force = self.twofg_get_max_force()
        self.max_ext_width = self.twofg_get_max_external_width()
        self.max_int_width = self.twofg_get_max_internal_width()
        self.min_ext_width = self.twofg_get_min_external_width()
        self.min_int_width = self.twofg_get_min_internal_width()

        self.gripper_width = [self.twofg_get_external_width(), self.twofg_get_internal_width()]

    
    def _send_xml_rpc_request(self, _req=None):

        headers = ["Content-Type: application/x-www-form-urlencoded"]

        data = _req.replace('\r\n','').encode()

        # Create a new cURL object
        curl = pycurl.Curl()

        # Set the URL to fetch
        curl.setopt(curl.URL, f'http://{self.robot_ip}:41414')
        curl.setopt(curl.HTTPHEADER, headers)
        curl.setopt(curl.POSTFIELDS, data)
        # Create a BytesIO object to store the response
        buffer = BytesIO()
        curl.setopt(curl.WRITEDATA, buffer)

        # Perform the request
        curl.perform()

        # Get the response body
        response = buffer.getvalue()

        # Print the response
        print(response.decode('utf-8'))

        # Close the cURL object
        curl.close()
        # Get response from xmlrpc server
        xml_response = xmlrpc.client.loads(response.decode('utf-8'))

        return xml_response[0][0]
    
    def twofg_get_external_width(self) -> float:
        xml_request = f"""<?xml version="1.0"?>
        <methodCall>
            <methodName>twofg_get_external_width</methodName>
                <params>
                    <param>
                        <value><int>{self.id}</int></value>
                    </param>
                </params>
        </methodCall>"""

        return float(self._send_xml_rpc_request(xml_request))
    
    def twofg_get_internal_width(self) -> float:
        xml_request = f"""<?xml version="1.0"?>
        <methodCall>
            <methodName>twofg_get_internal_width</methodName>
                <params>
                    <param>
                        <value><int>{self.id}</int></value>
                    </param>
                </params>
        </methodCall>"""

        return float(self._send_xml_rpc_request(xml_request))


    def twofg_grip_external(self, target_width: float = 40.00, target_force: int= 20, speed: int=1) -> int:
        """
            speed is the range from 1 to 100. It represents the percentage of the maximum speed.
        """
        assert target_width <= self.max_ext_width and target_width >= self.min_ext_width, f'Target Width must be within the range [{self.min_ext_width},{self.max_ext_width}]'
        assert target_force <= self.max_force or target_force >= 20, f'Target force must be within the range [20,{self.max_force}]'

        # WARNING: params will be sent straight to electrical system with no error checking on robot!
        if (target_width > self.max_ext_width):
            target_width = self.max_ext_width
        if(target_width < self.min_ext_width):
            target_width = self.min_ext_width
        if(target_force > self.max_force):
            target_force = self.max_force
        if(target_force < 20):
            target_force = 20

        xml_request = f"""<?xml version="1.0"?>
        <methodCall>
        <methodName>twofg_grip_external</methodName>
            <params>
                <param>
                    <value><int>{self.id}</int></value>
                </param>
                <param>
                    <value><double>{target_width}</double></value>
                </param>
                <param>
                    <value><int>{target_force}</int></value>
                </param>
                <param>
                    <value><int>{speed}</int></value>
                </param>
            </params>
        </methodCall>"""
        
        # if status != 0, then command not succesful. Perhaps there is no space to move the gripper
        return int(self._send_xml_rpc_request(xml_request))
    
    def twofg_ext_release(self, target_width: float = 40.00, speed: int=1) -> int:
        """
            speed is the range from 1 to 100. It represents the percentage of the maximum speed.
        """
        target_force: int= 80

        assert target_width <= self.max_ext_width and target_width >= self.min_ext_width, f'Target Width must be within the range [{self.min_ext_width},{self.max_ext_width}]'
        assert target_force <= self.max_force or target_force >= 20, f'Target force must be within the range [20,{self.max_force}]'

        # WARNING: params will be sent straight to electrical system with no error checking on robot!
        if (target_width > self.max_ext_width):
            target_width = self.max_ext_width
        if(target_width < self.min_ext_width):
            target_width = self.min_ext_width
        if(target_force > self.max_force):
            target_force = self.max_force
        if(target_force < 20):
            target_force = 20

        xml_request = f"""<?xml version="1.0"?>
        <methodCall>
        <methodName>twofg_grip_external</methodName>
            <params>
                <param>
                    <value><int>{self.id}</int></value>
                </param>
                <param>
                    <value><double>{target_width}</double></value>
                </param>
                <param>
                    <value><int>{target_force}</int></value>
                </param>
                <param>
                    <value><int>{speed}</int></value>
                </param>
            </params>
        </methodCall>"""
        
        # if status != 0, then command not succesful. Perhaps there is no space to move the gripper
        return int(self._send_xml_rpc_request(xml_request))
    
    def twofg_grip_internal(self, target_width: float = 40.00, target_force: int= 10, speed: int=1) -> int:
        """
            speed is the range from 1 to 100. It represents the percentage of the maximum speed.
        """
        assert target_width <= self.max_int_width and target_width >= self.min_int_width, f'Target Width must be within the range [{self.min_int_width},{self.max_int_width}]'
        assert target_force <= self.max_force or target_force >= 20, f'Target force must be within the range [20,{self.max_force}]'

        # WARNING: params will be sent straight to electrical system with no error checking on robot!
        if (target_width > self.max_int_width):
            target_width = self.max_int_width
        if(target_width < self.min_int_width):
            target_width = self.min_int_width
        if(target_force > self.max_force):
            target_force = self.max_force
        if(target_force < 20):
            target_force = 20

        xml_request = f"""<?xml version="1.0"?>
        <methodCall>
        <methodName>twofg_grip_internal</methodName>
            <params>
                <param>
                    <value><int>{self.id}</int></value>
                </param>
                <param>
                    <value><double>{target_width}</double></value>
                </param>
                <param>
                    <value><int>{target_force}</int></value>
                </param>
                <param>
                    <value><int>{speed}</int></value>
                </param>
            </params>
        </methodCall>"""
        
        # if status != 0, then command not succesful. Perhaps there is no space to move the gripper
        return int(self._send_xml_rpc_request(xml_request))
    
    def twofg_int_release(self, target_width: float = 40.00, speed: int=1) -> int:
        """
            speed is the range from 1 to 100. It represents the percentage of the maximum speed.
        """
        target_force: int= 80

        assert target_width <= self.max_int_width and target_width >= self.min_int_width, f'Target Width must be within the range [{self.min_int_width},{self.max_int_width}]'
        assert target_force <= self.max_force or target_force >= 20, f'Target force must be within the range [20,{self.max_force}]'

        # WARNING: params will be sent straight to electrical system with no error checking on robot!
        if (target_width > self.max_int_width):
            target_width = self.max_int_width
        if(target_width < self.min_int_width):
            target_width = self.min_int_width
        if(target_force > self.max_force):
            target_force = self.max_force
        if(target_force < 20):
            target_force = 20

        xml_request = f"""<?xml version="1.0"?>
        <methodCall>
        <methodName>twofg_grip_internal</methodName>
            <params>
                <param>
                    <value><int>{self.id}</int></value>
                </param>
                <param>
                    <value><double>{target_width}</double></value>
                </param>
                <param>
                    <value><int>{target_force}</int></value>
                </param>
                <param>
                    <value><int>{speed}</int></value>
                </param>
            </params>
        </methodCall>"""
        
        # if status != 0, then command not succesful. Perhaps there is no space to move the gripper
        return int(self._send_xml_rpc_request(xml_request))
    
    def twofg_get_max_external_width(self) -> float:
        xml_request = f"""<?xml version="1.0"?>
        <methodCall>
            <methodName>twofg_get_max_external_width</methodName>
                <params>
                    <param>
                        <value><int>{self.id}</int></value>
                    </param>
                </params>
        </methodCall>"""

        return float(self._send_xml_rpc_request(xml_request))
    
    def twofg_get_max_internal_width(self) -> float:
        xml_request = f"""<?xml version="1.0"?>
        <methodCall>
            <methodName>twofg_get_max_internal_width</methodName>
                <params>
                    <param>
                        <value><int>{self.id}</int></value>
                    </param>
                </params>
        </methodCall>"""

        return float(self._send_xml_rpc_request(xml_request))
    
    def twofg_get_min_external_width(self) -> float:
        xml_request = f"""<?xml version="1.0"?>
        <methodCall>
            <methodName>twofg_get_min_external_width</methodName>
                <params>
                    <param>
                        <value><int>{self.id}</int></value>
                    </param>
                </params>
        </methodCall>"""

        return float(self._send_xml_rpc_request(xml_request))
    
    def twofg_get_min_internal_width(self) -> float:
        xml_request = f"""<?xml version="1.0"?>
        <methodCall>
            <methodName>twofg_get_min_internal_width</methodName>
                <params>
                    <param>
                        <value><int>{self.id}</int></value>
                    </param>
                </params>
        </methodCall>"""

        return float(self._send_xml_rpc_request(xml_request))
    
    def twofg_get_max_force(self) -> int:
        xml_request = f"""<?xml version="1.0"?>
        <methodCall>
            <methodName>twofg_get_max_force</methodName>
                <params>
                    <param>
                        <value><int>{self.id}</int></value>
                    </param>
                </params>
        </methodCall>"""

        return int(self._send_xml_rpc_request(xml_request))
    
    def twofg_get_status(self) -> int:
        # The status codes are not fully clear.
        # sofar:
        # 0: no grip
        # 2: has gripped an object 

        xml_request = f"""<?xml version="1.0"?>
        <methodCall>
            <methodName>twofg_get_status</methodName>
                <params>
                    <param>
                        <value><int>{self.id}</int></value>
                    </param>
                </params>
        </methodCall>"""

        return int(self._send_xml_rpc_request(xml_request))
    
    def twofg_get_busy(self) -> bool:
        xml_request = f"""<?xml version="1.0"?>
        <methodCall>
            <methodName>twofg_get_busy</methodName>
                <params>
                    <param>
                        <value><int>{self.id}</int></value>
                    </param>
                </params>
        </methodCall>"""

        return bool(self._send_xml_rpc_request(xml_request))
    
    def twofg_get_grip_detected(self) -> bool:
        xml_request = f"""<?xml version="1.0"?>
        <methodCall>
            <methodName>twofg_get_grip_detected</methodName>
                <params>
                    <param>
                        <value><int>{self.id}</int></value>
                    </param>
                </params>
        </methodCall>"""

        return bool(self._send_xml_rpc_request(xml_request))
       
def main():

    # Default id is zero, if you have multiple grippers, 
    # see logs in UR Teach Pendant to know which is which :)
    print("Main")
    rg_id = 0
    master_ip = "192.168.5.4"
    slave_ip = "192.168.5.5"

    master_gripper = RG2(master_ip, rg_id)
    slave_gripper = RG2(slave_ip, rg_id)

    print(f"Master gripper initial width: {master_gripper.get_rg_width()}")
    print(f"Slave gripper initial width: {slave_gripper.get_rg_width()}")
    
    while True:
        command_str = input("Enter command ('master open', 'slave close', or 'exit'): ").lower().strip()
        
        if command_str == 'exit':
            break

        parts = command_str.split()
        if len(parts) != 2:
            print("Invalid command. Use format: <arm> <action> (e.g., 'master open')")
            continue

        arm, action = parts
        
        target_gripper = None
        if arm == 'master':
            target_gripper = master_gripper
        elif arm == 'slave':
            target_gripper = slave_gripper
        else:
            print("Invalid arm. Use 'master' or 'slave'.")
            continue

        if action == 'open':
            target_gripper.rg_open()
            print(f"{arm} gripper open command sent.")
        elif action == 'close':
            target_gripper.rg_close()
            print(f"{arm} gripper close command sent.")
        else:
            print("Invalid action. Use 'open' or 'close'.")


if __name__ == "__main__":
    main()