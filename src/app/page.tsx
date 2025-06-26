"use client";

import { useRosListener } from "@/hooks/useRosListener";
import { KinematicsData } from "@/types/KinematicsData";
import { useCallback, useState } from "react";
import styles from "./page.module.scss";
import { initialData } from "./initialData";

export default function Home() {
  const [snapshot, setSnapshot] = useState<KinematicsData>(initialData);

  const handleKinematics = useCallback((data: KinematicsData) => {
    setSnapshot(data);
  }, []);

  useRosListener(handleKinematics);

  return (
    <main className={styles.all}>
      <h2>End Effector</h2>
      <table>
        <tr>
          <td>
            <div className={styles.cellLabel}>∂<sub>x</sub></div>
            <div className={styles.cellValue}>{snapshot?.rot[0].toFixed(2)}</div>
          </td>
          <td>
            <div className={styles.cellLabel}>∂<sub>y</sub></div>
            <div className={styles.cellValue}>{snapshot?.rot[1].toFixed(2)}</div>
          </td>
          <td>
            <div className={styles.cellLabel}>∂<sub>z</sub></div>
            <div className={styles.cellValue}>{snapshot?.rot[2].toFixed(2)}</div>
          </td>

          <td>
            <div className={styles.cellLabel}>∂θ<sub>x</sub></div>
            <div className={styles.cellValue}>{snapshot?.rot[0].toFixed(2)}</div>
          </td>
          <td>
            <div className={styles.cellLabel}>∂θ<sub>y</sub></div>
            <div className={styles.cellValue}>{snapshot?.rot[1].toFixed(2)}</div>
          </td>
          <td>
            <div className={styles.cellLabel}>∂θ<sub>z</sub></div>
            <div className={styles.cellValue}>{snapshot?.rot[2].toFixed(2)}</div>
          </td>
        </tr>
      </table>
      <h2>Joints</h2>
      <table>
        <tr>
          {snapshot?.q_in.map((val, idx) => (
            <td key={idx}>
              <div className={styles.cellLabel}>
                θ<sub>{idx + 1}</sub>
              </div>
              <div className={styles.cellValue}>{val.toFixed(2)}</div>
            </td>
          ))}
        </tr>
      </table>
      <h2>Jacobian</h2>
      <div className={styles.jacobian}>
        <svg width="32" className={styles.jacobianOpen} viewBox="0 0 58 792">
          <polygon fill="white" points="6 23.05 15.6 23.05 15.6 25.03 6 25.03 6 766.97 15.6 766.97 15.6 768.95 6 768.95 1.48 768.95 1.48 23.05 6 23.05"/>
        </svg>
        <table>
          <tr>
            <td>
              <div className={[styles.cellLabel, styles.callLabelPre].join(' ')}>
                ∂p<sub>x</sub>
              </div>
              <div className={styles.cellValue}>{0}</div>
            </td>
          </tr>
        </table>
        <svg width="32" className={styles.jacobianClose} viewBox="0 0 58 792">
          <polygon fill="white" points="53.48 23.05 43.88 23.05 43.88 25.03 53.48 25.03 53.48 766.97 43.88 766.97 43.88 768.95 53.48 768.95 58 768.95 58 23.05 53.48 23.05"/>
        </svg>
      </div>
    </main>
  );
}
