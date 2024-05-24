from copy import deepcopy

from pprint import pprint

NAME_PARSER = {
    'act_clients': "Action Clients",
    'act_services': "Action Services",
    'pub': "Publishers",
    'srv_clients': "Server Clients",
    'srv_services': "Servers Services",
    'sub': "Subscribers",
    'name': "Name"
}

# FORMAT_TEMPLATE = "\n\t{: <10}{: ^10}{: ^10}{: >5}\n"
FORMAT_TEMPLATE = "{:<10} {:<50} {:>40}\n"


class DataFormatter:

    _remove_hidden_nodes: bool 

    def __init__(self, remove_hidden_nodes=True):
        self._remove_hidden_nodes = remove_hidden_nodes

    def format(self, data: dict):
        res_data = deepcopy(data)
        
        if self._remove_hidden_nodes:
            for node in list(res_data.keys()): 
                if node[:2] == '/_':
                    del res_data[node]

        output_str = ""
        # # printing Aligned Header 
        output_str += FORMAT_TEMPLATE.format(" "," "," ")
        
        # printing values of variables in Aligned manner 
        # for i in range(0, 4): 
            # print(f"{names[i] : <10}{marks[i] : ^10}{div[i] : ^10}{id[i] : >5}") 
        for k,v in res_data.items():
            output_str += self.format_node(v)
            output_str += "---"
        # for key, value in list(res_data.items()):
        #     # print(key)
        #     # pprint(value)
        #     for data_type, data in value.items():
        #         if data_type == "name":
        #             output_str += FORMAT_TEMPLATE.format(NAME_PARSER[data_type], data, " ")
        #         else:
        #             output_str += FORMAT_TEMPLATE.format(NAME_PARSER[data_type], " ", " ")
        #             output_str += self.format_interface_data(data)
        #     # print(output_str)
        #     # # print(self.replace_empty_list('sub', value['sub']))
        #     # exit()
        return output_str
    

    def format_node(self, node:dict):
        output_str = ""
        output_str += FORMAT_TEMPLATE.format(" "," "," ")
        # print(node)
        # input()
        for data_type, data in list(node.items()):
            if data_type == "name":
                output_str += FORMAT_TEMPLATE.format(NAME_PARSER[data_type], " ", " ")
                output_str += FORMAT_TEMPLATE.format(" ", data, " ")
            else:
                output_str += FORMAT_TEMPLATE.format(NAME_PARSER[data_type], " ", " ")
                output_str += self.format_interface_data(data)
        return output_str
    
    @staticmethod
    def format_interface_data(data):
        output_str = ""
        if isinstance(data, list):
            # output_str += "\n{} :".format(data_type)
            for d in data:
                output_str += FORMAT_TEMPLATE.format(" ", d[0], d[1][0])
            return output_str
        return output_str

        # if len(data) == 0: 
        #     if isinstance(data, list): 
        #         output_str += FORMAT_TEMPLATE.format()
        #         return '\n{} : N/A'.format(data_type)
        # else: 
        #     if isinstance(data, list):
        #         output_str += "\n{} :".format(data_type)
        #         for d in data:
        #             output_str+='\n\t{} [ {} ]'.format(d[0], d[1][0])
        #         return output_str
        # return '\n'