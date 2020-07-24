from xml.dom import minidom
import re
import numpy as np

class URDF:

    joints = []
    links = []
    transmissions = []
    props = []
    attr = []
    jseq = []

    def __init__(self, file_name):
        
        dom = minidom.parse(file_name)

        self.links = dom.getElementsByTagName('link')
        self.joints = dom.getElementsByTagName('joint')
        self.properties = dom.getElementsByTagName('property')
        self.robot = dom.getElementsByTagName('robot')
        self.attr = self.getattr(self.robot.item(0),{},[])

        # get the properties
        #print(f'{len(self.properties)} properties')
        Props = []
        for k in range(len(self.properties)):
            property = self.properties.item(k)
            if len(property.attributes) > 0:
                attributes = property.getAttributes
                
                att = {}
                for i in range(len(attributes)):
                    
                    attribute = attributes.item(i-1)
                    n = attribute.name
                    v = attribute.value
                    att[n] = v

                Props = [[Props],[att]]
        
        self.props = Props

        # get the joints
        self.joints = self.get_elements(dom, 'joint', Props)

        # get the links
        self.links = self.get_elements(dom, 'link', Props)
        
        # get the transmissions
        self.transmissions = self.get_elements(dom, 'transmission', Props)
        
        # get the robot
        self.robot = self.get_elements(dom, 'robot', Props)

        p = np.zeros((1, len(self.links)))[0]
        for j in range(len(self.joints)):
            i = self.ln2i(self.joints[j]['parent']['link'])
            p[i] = p[i] + 1
            i = self.ln2i(self.joints[j]['child']['link'])
            p[i] = p[i] - 1

        base_link = np.argmax(p)
        base_link_name = self.links[base_link]['name']
        link = base_link_name
        self.jseq = [None]*len(self.joints)
        for j in range(len(self.joints)):
            jj = self.findnextjoint(link) # --------------
            if jj == []:
                break
            self.jseq[j] = jj
            link = self.joints[jj]['child']['link']
        
        #self.display()
        
    # def robot(self):
    #     for i in range(self.njoints()):
    #         # create link objects
    #         # attach the STL model to them
    #         pass
    #     return SerialLink(links, 'name', self.attr['name']);

    def nlinks(self):
        return len(self.links)

    def njoints(self):
        return len(self.joints)

    def findnextjoint(self,link):
        joint = []
        for j in range(len(self.joints)):
            if link == self.joints[j]['parent']['link']:
                joint = j
                return joint
        return joint

    def ln2i(self,name):
        idx = []
        for i in range(len(self.links)):
            if self.links[i]['name'] == name:
                idx = i
                return idx
        return idx

    def display(self):
        for j in range(self.njoints()):
            joint = self.joints[j]
            print(f"j{j+1}: {joint['parent']['link']} -> {joint['child']['link']} ({joint['type']})")

    def get_elements(self, doc, elname, props):
        elements = doc.getElementsByTagName(elname)
        # get the links
        List = []
        #print(f'{len(elements)} {elname}')
        # step through the list of  elements found
        for k in range(len(elements)):
            element = elements.item(k)

            info = self.descend(element, props)
            info = self.getattr(element, info, props)
            List.append(info)

        return List

    def descend(self, node, props):
        if node.hasChildNodes():
            t = {}

            nodeChildren = node.childNodes
            for i in range(len(nodeChildren)):
                child = nodeChildren.item(i)
                if child.nodeType == 3:
                    continue
                result = self.descend(child, props)
                if len(result) > 0:
                    n = child.tagName
                    t[n] = result
        elif len(node.attributes) > 0:
            t = self.getattr(node, {}, props)
        else:
            t = {}
        
        return t
    def getattr(self, node, att, props):
        attributes = node.attributes

        for i in range(len(attributes)):
            attribute = attributes.item(i-1)
            n = attribute.name
            v = attribute.value

            # skip properties with colon in them
            n.replace(':','_')

            # do simple xacro type substitution
            # xyz="0 0 ${-base_height}"
            if len(props) > 0:
                # is ther a ${...} substrings
                (s,e) = re.compile(r'\${[^}]*').match(v).span()

                print(s,e)
            
            try:
                att[n] = [float(val) for val in v.split(' ')]
            except:
                att[n] = v
            

        return att
    
    def __str__(self):
        s = ""
        for j in range(self.njoints()):
            joint = self.joints[j]
            s += f"j{j+1}: {joint['parent']['link']} -> {joint['child']['link']} ({joint['type']})\n"
        return s


if __name__ == "__main__":
    robot = URDF('D:\\cours\\mea4\\Stage\\hoap3_description-master\\hoap3.urdf')