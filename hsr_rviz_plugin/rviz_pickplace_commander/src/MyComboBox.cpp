#include "MyComboBox.h"
 
MyComboBox::MyComboBox(QWidget *parent):QComboBox(parent)
{
 
}
 
MyComboBox::~MyComboBox()
{
 
}
 
void MyComboBox::mousePressEvent(QMouseEvent *event)
{
    if(event->button() == Qt::LeftButton)
    {
        Q_EMIT clicked(true);  //触发clicked信号
    }
 
    QComboBox::mousePressEvent(event);  //将该事件传给父类处理，这句话很重要，如果没有，父类无法处理本来的点击事件
}
