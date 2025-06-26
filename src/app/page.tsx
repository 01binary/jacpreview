"use client";

import { useRosListener } from "@/hooks/useRosListener";
import { KinematicsData } from "@/types/KinematicsData";
import { useCallback, useState } from "react";
import styles from "./page.module.scss";
import { initialData } from "./initialData";

const MAX_COLOR = { r: 240, g: 96, b: 54 };
const MIN_COLOR = { r: 249, g: 183, b: 51 };

const getColorForValue = (value: number) => {
  if (value > 1) {
    console.log('greater than one!', value)
  }
  const r = Math.round(MIN_COLOR.r + (MAX_COLOR.r - MIN_COLOR.r) * value);
  const g = Math.round(MIN_COLOR.g + (MAX_COLOR.g - MIN_COLOR.g) * value);
  const b = Math.round(MIN_COLOR.b + (MAX_COLOR.b - MIN_COLOR.b) * value);
  return `rgb(${r}, ${g}, ${b})`;
};

export default function Home() {
  const [snapshot, setSnapshot] = useState<KinematicsData>(initialData);

  const handleKinematics = useCallback((data: KinematicsData) => {
    setSnapshot(data);
  }, []);

  useRosListener(handleKinematics);

  return (
    <main className={styles.all}>
      <table>
        <tr>
          <td>
            <div className={styles.cellLabel}><span className={styles.unimportant}>∂</span>p<sub>x</sub></div>
            <div className={styles.cellValue}>{Math.abs(snapshot?.vel[0]).toFixed(2)}</div>
          </td>
          <td>
            <div className={styles.cellLabel}><span className={styles.unimportant}>∂</span>p<sub>y</sub></div>
            <div className={styles.cellValue}>{Math.abs(snapshot?.vel[1]).toFixed(2)}</div>
          </td>
          <td>
            <div className={styles.cellLabel}><span className={styles.unimportant}>∂</span>p<sub>z</sub></div>
            <div className={styles.cellValue}>{Math.abs(snapshot?.vel[2]).toFixed(2)}</div>
          </td>

          <td>
            <div className={styles.cellLabel}><span className={styles.unimportant}>∂</span>r<sub>x</sub></div>
            <div className={styles.cellValue}>{Math.abs(snapshot?.rot[0]).toFixed(2)}</div>
          </td>
          <td>
            <div className={styles.cellLabel}><span className={styles.unimportant}>∂</span>r<sub>y</sub></div>
            <div className={styles.cellValue}>{Math.abs(snapshot?.rot[1]).toFixed(2)}</div>
          </td>
          <td>
            <div className={styles.cellLabel}><span className={styles.unimportant}>∂</span>r<sub>z</sub></div>
            <div className={styles.cellValue}>{Math.abs(snapshot?.rot[2]).toFixed(2)}</div>
          </td>
        </tr>
      </table>
  
      <table>
        <tr>
          {snapshot?.q_in.map((val, idx) => (
            <td key={idx}>
              <div className={styles.cellLabel}>
                θ<sub>{idx + 1}</sub>
              </div>
              <div className={styles.cellValue}>
                {(val >= 0 ? ' ' : '') + val.toFixed(2)}
              </div>
            </td>
          ))}
        </tr>
      </table>

      <div className={styles.jacobian}>
        <table>
          {['px', 'py', 'pz', 'rx', 'ry', 'rz'].map((endEffectorProperty, endEffectorPropertyIndex) => (
            <tr>
              {[1, 2, 3, 4, 5, 6, 7].map((jointIndex) => (
                <td style={{
                  color: getColorForValue(snapshot[`r${endEffectorPropertyIndex + 1}`][jointIndex - 1])
                }}>
                  <div className={styles.fraction}>
                    <div className={styles.numerator}>
                      <span className={styles.unimportant}>∂</span>{endEffectorProperty[0]}<sub>{endEffectorProperty[1]}</sub>
                    </div>
                    <div className={styles.denominator}>
                      <span className={styles.unimportant}>∂</span>θ<sub>{jointIndex}</sub>
                    </div>
                  </div>
                </td>
              ))}
            </tr>
          ))}
        </table>
      </div>
    </main>
  );
}
