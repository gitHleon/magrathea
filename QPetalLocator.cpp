/*
 * QPetalLocator.cpp
 *
 *  Created on: Jul 26, 2019
 *      Author: lacasta
 */
#include <iostream>
#include <QtWidgets>
#include "QPetalLocator.h"

struct QPetalLocatorPriv
{
        QLabel *intro;
        QLabel *loc1, *loc2;
        QPushButton *btn_loc1, *btn_loc2;
        QDialogButtonBox *buttonBox;

};

QPetalLocator::QPetalLocator(MotionHandler *M)
    : motion(M), priv(0)
{
    priv = new QPetalLocatorPriv;
    priv->intro = new QLabel("Navigate to the position of the petal locators and click the button when ready");
    priv->loc1 = new QLabel("Top Petal locator");
    priv->loc2 = new QLabel("Bottom Petal locator");
    priv->btn_loc1 = new QPushButton("Set");
    priv->btn_loc2 = new QPushButton("Set");

    connect(priv->btn_loc1, SIGNAL (clicked()), this, SLOT (find_top_locator()));
    connect(priv->btn_loc2, SIGNAL (clicked()), this, SLOT (find_bottom_locator()));

    priv->buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok
                                         | QDialogButtonBox::Cancel);

    connect(priv->buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(priv->buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);

    QVBoxLayout *V = new QVBoxLayout;
    V->addWidget(priv->intro);

    QGridLayout *grid = new QGridLayout;
    grid->addWidget(priv->loc1, 0, 0, 1, 1);
    grid->addWidget(priv->loc2, 1, 0, 1, 1);
    grid->addWidget(priv->btn_loc1, 0, 1, 1, 1);
    grid->addWidget(priv->btn_loc2, 1, 1, 1, 1);
    V->addLayout(grid);

    V->addWidget(priv->buttonBox);
    setLayout(V);
}

QPetalLocator::~QPetalLocator()
{
    delete priv;
}

void QPetalLocator::find_top_locator()
{
    std::cout << "Getting position of top locator " << std::endl;
    std::vector<double> current_pos( motion->whereAmI() );
    top.x(current_pos[0]);
    top.y(current_pos[1]);
}
void QPetalLocator::find_bottom_locator()
{
    std::cout << "Getting position of bottom locator " << std::endl;
    std::vector<double> current_pos( motion->whereAmI() );
    bottom.x(current_pos[0]);
    bottom.y(current_pos[1]);
}
