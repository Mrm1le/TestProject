#include <iostream>
template <typename T>
class listItem {
public:
    listItem(const T& value) :_value(value), _next(nullptr) {}
    bool operator==(const listItem& rhs) { return _value = rhs._value; }
    bool operator==(const T& value) { return value == _value; }
    bool operator!=(const listItem& rhs) { return _value != rhs._value; };
    bool operator!=(const T& value) { return value != _value; }
    void setNext(listItem* item) { _next = item; }
    T& getValue() { return _value; }
    listItem* getNext() { return _next; }
private:
    T _value;
    listItem* _next;
};

template <typename T>
class list {
public:
    void insertFront(const T& value);
    void insertBack(const T& value);
    list() :_size(0), _begin(nullptr), _end(nullptr) {}
    unsigned long size() { return _size; }
    T front() { return _begin->getValue(); }
    listItem<T>* begin() { return _begin; }
    void display(std::ostream& os = std::cout);
private:
    unsigned long _size;
    listItem<T>* _begin;
    listItem<T>* _end;
};

template <typename Item>
class listIter {
public:
    listIter& operator++();
    listIter operator++(int);
    Item& operator*() { return *_item; }
    Item* operator->() { return _item; }
    bool operator==(const listIter& rhs);
    bool operator!=(const listIter& rhs);
    listIter() :_item(nullptr) {}
    listIter(Item* item) :_item(item) {}
private:
    Item* _item;
};

template <typename Item, typename T>
listIter<Item> find(listIter<Item> begin, listIter<Item> end, const T& value) {
    while (begin != end && *begin != value)
        ++begin;
    return begin;
}

template <typename T>
void list<T>::insertBack(const T& value) {
    if (!_begin) {
        insertFront(value);
        return;
    }
    auto last = _begin;
    while (last->getNext() != _end)
        last = last->getNext();
    auto temp = new listItem<T>(value);
    last->setNext(temp);
    ++_size;
}

template <typename T>
void list<T>::insertFront(const T& value) {
    auto temp = new listItem<T>(value);
    temp->setNext(_begin);
    _begin = temp;
    ++_size;
}

template <typename Item>
bool listIter<Item>::operator==(const listIter<Item>& rhs) {
    return this->_item == rhs._item;
}

template <typename Item>
bool listIter<Item>::operator!=(const listIter<Item>& rhs) {
    return this->_item != rhs._item;
}

template <typename Item>
listIter<Item>& listIter<Item>::operator++() {
    this->_item = this->_item->getNext();
    return *this;
}

template <typename Item>
listIter<Item> listIter<Item>::operator++(int) {
    auto temp = *this;
    this->_item = this->_item->getNext();
    return temp;
}

template <typename T>
void list<T>::display(std::ostream& os) {
    os << "list: " << std::endl;
    auto last = _begin;
    while (last != _end) {
        os << last->getValue() << std::endl;
        last = last->getNext();
    }
}



