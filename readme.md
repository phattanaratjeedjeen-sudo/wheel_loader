#  Static Load Estimation for Wheel Loader


## Table of Contents
- [Wheel Loader Model](#wheel-loader-model)
- [Reference](#reference)


## Wheel Loader Model

<table align="center" style="border:none;">
  <tr>
    <td align="center" width="50%">
      <img src="./material/model_overall.png" alt="overall model" width="100%"/>
      <br>
      <em>Overall model</em>
    </td>
    <td align="center" width="50%">
      <img src="./material/model_detail.png" alt="detail model" width="88%"/>
      <br>
      <em>Detail model</em>
    </td>
  </tr>
</table>

### Variables
#### Load Calculation Variables
| Variable | Description | Unit |
| :--- | :--- | :--- |
| **n** | number of cylinders | - |
| **Ab** | bottom side pressure receiving area of cylinder | cm² |
| **Pb** | bottom pressure of cylinder | MPa |
| **Ar** | rod side pressure receiving area of cylinder | cm² |
| **Pr** | rod pressure of cylinder | MPa |
| **M** | moment around lift arm hinge pin | Nm |
| **Fc** | force applied to lift arm cylinder | N |
| **Fb** | force applied to bucket cylinder | N |
| **f** | horizontal length between hinge pin and vector of Fc | m |
| **e** | horizontal length between hinge pin and vector of Fb | m |
| **a** | horizontal length between load center and bucket pin hinge pin | m |
| **b** | distance between bucket pins | m |
| **c** | distance between push rod center length and bell crank center pin | m |
| **d** | distance between bell crank pins | m |
| **M1** | moment around lift arm hinge pin in loaded state | Nm |
| **M0** | moment around lift arm hinge pin in unloaded state | Nm |
| **W** | load | kg |
| **Lw** | horizontal length from gravity center position of load | - |

#### Dimension & Linkage Posture Variabless
| Variable | Description | Unit |
| :--- | :--- | :--- |
| **θg** | Lift arm angle ∠(horizontal, Lag) | deg |
| **Ldf** | Length between bell crank D pin and bucket cylinder root F pin | mm |
| **Ldg** | Length between bell crank D pin and lift arm hinge G pin | mm |
| **Lfg** | Length between bucket cylinder root F pin and lift arm hinge G pin | mm |
| **LfgX** | Horizontal length between bucket cylinder root F pin and lift arm hinge G pin | mm |
| **LfgY** | Vertical length between bucket cylinder root F pin and lift arm hinge G pin | mm |
| **∠DGA** | ∠(Ldg, Lag) | deg |
| **Laf** | Length between lift arm tip A pin and bucket cylinder root F pin | mm |
| **Lag** | Length between lift arm tip A pin and hinge G pin | mm |
| **∠FGO** | ∠(Lfg, Horizontal) | deg |
| **θf** | Bucket cylinder posture angle ∠(horizontal, Lef) | deg |
| **Lef** | Bucket cylinder stroke length | mm |
| **θe** | Bell crank posture angle ∠(Lde, Lef) | deg |
| **Lde** | Bell crank DE pin length | mm |
| **∠ADC** | ∠(Lad, Lcd) | deg |
| **Lad** | Length between lift arm tip A pin and center D pin | mm |
| **θd** | ∠(Lde, Lcd) | deg |
| **Lac** | Length between lift arm tip A pin and center C pin | mm |
| **Lcd** | Bell crank CD pin length | mm |
| **θc** | ∠(Lcd, Lbc) | deg |
| **Lbc** | Distance between push rod BC pins | mm |
| **Lab** | Distance between bucket AB pins | mm |
| **θb** | ∠(Lbc, Lab) | deg |
| **LloadG**| load center length | - |
| **HloadG**| load center angle | - |
| **Hbmcyl**| lift arm cylinder angle | - |

## Reference
* **Patent:** US 2020/0131739 A1 (Hitachi Construction Machinery, Apr. 30, 2020). 