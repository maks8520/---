// Насадка под Supra VCS-1832
$fn = 100;

// Размеры корпуса
id_tube = 32;       // Внутренний диаметр
od_tube = 42;       // Внешний диаметр
len_tube = 22;      // Общая длина корпуса
wall = 3;           // Толщина стенки корпуса (минимальная 2.5-3)

// Размеры рычага и фиксатора
lever_length = 35;
axis_slot_w = 6;
axis_d = 5.7;
bump_h = 4;
bump_w = 8;
slot_w = 9;
slot_d = 4.5;

module housing() {
    difference() {
        // Основной корпус
        cylinder(d=od_tube, h=len_tube, center=true);

        // Внутреннее отверстие под трубу
        cylinder(d=id_tube, h=len_tube + 2, center=true);

        // Паз под ось рычага (6 мм ширина) - сверху
        translate([od_tube/2 - 2, 0, len_tube/2])
            cube([10, axis_slot_w, 10], center=true);

        // Паз под фиксатор (ширина 9, глубина 4.5) - снизу рычага
        translate([od_tube/2 - slot_d/2 + 0.1, 0, -len_tube/4])
            cube([slot_d + 2, slot_w, 15], center=true);
    }

    // Крепления оси на корпусе
    translate([od_tube/2 - 1, 0, len_tube/2 - 2]) {
        difference() {
            cube([6, 14, 8], center=true);
            cube([7, axis_slot_w, 10], center=true);
            rotate([90, 0, 0]) cylinder(d=axis_d + 0.3, h=16, center=true);
        }
    }
}

module lever() {
    // Рычаг расположен в рабочем положении (собранный вид)
    translate([od_tube/2 + 1, 0, len_tube/2 - 2]) {

        // Ось рычага (5.7 мм)
        rotate([90, 0, 0]) cylinder(d=axis_d, h=axis_slot_w - 0.4, center=true);

        // Тело рычага (длина 35 мм)
        translate([2, 0, -lever_length/2 + 2]) {
            cube([3, axis_slot_w - 0.4, lever_length], center=true);

            // Выступ-фиксатор (высота 4, ширина 8)
            // Он должен входить в паз корпуса
            translate([-1.5 - bump_h/2, 0, lever_length/2 - 15]) {
                // bump_h = 4, bump_w = 8
                // Сделаем бугорок скругленным
                hull() {
                    cube([0.1, bump_w, bump_w], center=true);
                    translate([-bump_h, 0, 0]) cube([0.1, bump_w-2, bump_w-2], center=true);
                }
            }
        }
    }
}

// Сборка для визуализации
housing();
color("red") lever();
