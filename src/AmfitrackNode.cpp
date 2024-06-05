///-----------------------------------------------------------------------------
//                              AMFITECH APS
//                          ALL RIGHTS RESERVED
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include "AmfitrackNode.h"

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------
using namespace AMFITRACK_API_LIB;


AmfitrackNode::AmfitrackNode(uint8_t tx_id, uint32_t *uuid) : _tx_id(tx_id), _name_length(AmfiProtMaxPayloadLength - 1)
{
    _uuid[0] = uuid[0];
    _uuid[1] = uuid[1];
    _uuid[2] = uuid[2];
    
    _dev_name = new char[53];
    _dev_name[0] = '\0';
    
    _dev_handle = nullptr;
}

AmfitrackNode::AmfitrackNode(const char name[]) : _name_length(strlen(name))
{
    _dev_name = new char[53];
    strcpy(_dev_name, name);
    _dev_name[_name_length] = '\0';
    
    _dev_handle = nullptr;
}

AmfitrackNode::AmfitrackNode(hid_device *dev) : _dev_handle(dev), _name_length(AmfiProtMaxPayloadLength - 1)
{
    _dev_name = new char[53];
    _dev_name[0] = '\0';
}

AmfitrackNode::AmfitrackNode() : _name_length(AmfiProtMaxPayloadLength - 1)
{
    _dev_name = new char[53];
    _dev_name[0] = '\0';
    _dev_handle = nullptr;
}

AmfitrackNode::AmfitrackNode(const AmfitrackNode& node) : _tx_id(node.getTxID()), _name_length(node.getNameLength()), 
                                                        _dev_handle(node.getDeviceHandle()), _is_hub(node.isHub()),
                                                        _is_rf(node.isRF()), _is_usb(node.isUSB())
{
    _dev_name = new char[53];
    uint32_t *uuid = node.getUUID();
    _uuid[0] = uuid[0];
    _uuid[1] = uuid[1];
    _uuid[2] = uuid[2];

    strcpy(_dev_name, node.getDevName());
    _child_nodes = node.getChildNodes();
}

AmfitrackNode::~AmfitrackNode()
{

}

AmfitrackNode& AmfitrackNode::operator=(const AmfitrackNode& rhs)
{
    _tx_id = rhs.getTxID();
    uint32_t *uuid = rhs.getUUID();
    _uuid[0] = uuid[0];
    _uuid[1] = uuid[1];
    _uuid[2] = uuid[2];

    _name_length = rhs.getNameLength();
    strcpy(_dev_name, rhs.getDevName());

    _is_hub = rhs.isHub();
    _is_rf = rhs.isRF();
    _is_usb = rhs.isUSB();
    _dev_handle = rhs.getDeviceHandle();
    _child_nodes = rhs.getChildNodes();

    return *this;
}

bool AMFITRACK_API_LIB::operator==(const AmfitrackNode& lhs, const AmfitrackNode& rhs)
{
    bool ret;
    uint32_t *lhs_uuid, *rhs_uuid;
    lhs_uuid = lhs.getUUID();
    rhs_uuid = rhs.getUUID();
    ret = ( (lhs_uuid[0] == rhs_uuid[0]) && (lhs_uuid[1] == rhs_uuid[1]) && (lhs_uuid[2] == rhs_uuid[2]) );
    ret = ret && (lhs.getTxID() == rhs.getTxID());
    
    return ret;
}
bool AMFITRACK_API_LIB::operator!=(const AmfitrackNode& lhs, const AmfitrackNode& rhs)
{
    return !(operator==(lhs, rhs));
}

std::ostream& AMFITRACK_API_LIB::operator<<(std::ostream& os, const AmfitrackNode& node)
{
    uint32_t *uuid = node.getUUID();
    os << "TxID: " << (int)node.getTxID() << "\t UUID: " << uuid[0] << uuid[1] << uuid[2] << "\t name: " << node.getDevName();
    return os;
}

bool AmfitrackNode::contains(const AmfitrackNode& node)
{
    bool ret = false;
    if (this->isHub())
    {
        for (auto child_node : _child_nodes)
        {
            ret = ret || (*(child_node.get()) == node);
        }
    }
    else
    {
        return operator==(*this, node);
    }
    return ret;
}

std::shared_ptr<AmfitrackNode> AmfitrackNode::find_match(const AmfitrackNode& node) 
{
    if (this->isHub())
    {
        for (auto child_node : _child_nodes)
        {
            if (node == *child_node.get())
            {
                return child_node;
            }
        }
    }
    return NULL;
}

uint8_t AmfitrackNode::getTxID() const
{
    return _tx_id;
}

uint32_t* AmfitrackNode::getUUID() const
{
    return (uint32_t *)_uuid;
}

uint32_t AmfitrackNode::getNameLength() const
{
    return _name_length;
}

char *AmfitrackNode::getDevName() const
{
    return _dev_name;
}

hid_device *AmfitrackNode::getDeviceHandle() const
{
    return _dev_handle;
}

std::vector<std::shared_ptr<AmfitrackNode>> AmfitrackNode::getChildNodes() const
{
    return _child_nodes;
}

void AmfitrackNode::setTxID(uint8_t tx_id)
{
    _tx_id = tx_id;
}

void AmfitrackNode::setUUID(uint32_t *uuid)
{
    _uuid[0] = uuid[0];
    _uuid[1] = uuid[1];
    _uuid[2] = uuid[2];
}

void AmfitrackNode::setNameLength(uint32_t name_length)
{
    _name_length = name_length;
}

void AmfitrackNode::setDevName(const char name[], uint32_t name_length)
{
    assert(name_length <= _name_length);
    strcpy(_dev_name, name);
}

void AmfitrackNode::setDeviceHandle(hid_device *dev)
{
    _dev_handle = dev;
}

void AmfitrackNode::addChildNode(std::shared_ptr<AmfitrackNode> node)
{
    _child_nodes.push_back(node);
}


void AmfitrackNode::setHubMode(bool is_hub)
{
    _is_hub = is_hub;
}
bool AmfitrackNode::isHub() const
{
    return _is_hub;
}

void AmfitrackNode::setUSBMode(bool is_usb)
{
    _is_usb = is_usb;
}
bool AmfitrackNode::isUSB() const
{
    return _is_usb;
}

void AmfitrackNode::setRFMode(bool is_rf)
{
    _is_rf = is_rf;
}
bool AmfitrackNode::isRF() const
{
    return _is_rf;
}