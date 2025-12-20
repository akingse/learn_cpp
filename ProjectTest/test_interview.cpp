#include"pch.h"
using namespace std;

struct Node
{
    Node* m_prev = nullptr;
    Node* m_next = nullptr;
    int m_value;
};
class MyList
{
public:
    Node* m_node = nullptr;
    Node* back()
    {
        if (!m_node)
            return nullptr;
        Node* current = m_node;
        while (1)
        {
            if (current->m_next == nullptr)
                break;
            current = current->m_next;
        }
        return current;
    }
    void push_back(int v)
    {
        Node* node = new Node;
        node->m_value = v;
        if (m_node == nullptr)
            m_node = node;
        else
        {
            Node* _back = back();
            _back->m_next = node;
            node->m_prev = _back;
        }
    }
    void print()
    {
        Node* current = m_node;
        while (1)
        {
            cout << "value=" << current->m_value << endl;
            if (current->m_next == nullptr)
                break;
            current = current->m_next;
        }
    }

    void reverse(int k = 3)
    {
        if (!m_node)
            return;
        Node* current = m_node;
        Node* _back = back();

        while (current != _back && current!=nullptr)
        {
            std::vector<int> store;
            int count = 0;
            Node* temp = current;
            while (count < k)
            {
                if (current == nullptr)
                    break;
                store.push_back(current->m_value);
                current = current->m_next;
                count++;
            }
            std::reverse(store.begin(), store.end());
            if (store.size() < k)
                continue;
            for (int i = 0; i < k; i++)
            {
                temp->m_value = store[i];
                temp = temp->m_next;
            }
            store.clear();
        }

    }


};

static void _test0()
{
    //std::list<int> list;
    MyList alist;
    alist.push_back(1);
    alist.push_back(2);
    alist.push_back(3);
    alist.push_back(4);
    alist.push_back(5);
    alist.push_back(6);

    alist.reverse(6);
    alist.print();
    return;

}


static int enrol = []()->int
    {
        //_test0();
        cout << clash::get_filepath_filename(__FILE__) << " finished.\n" << endl;
        return 0;
    }();

